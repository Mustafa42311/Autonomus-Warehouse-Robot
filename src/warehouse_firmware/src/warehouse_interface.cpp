#include "warehouse_firmware/warehouse_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>


namespace warehouse_firmware
{
WarehouseInterface::WarehouseInterface()
    : goal_success_trigger_(0.0)
{
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "WarehouseInterface constructor called");
}


WarehouseInterface::~WarehouseInterface()
{
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "WarehouseInterface destructor called");
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
      RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Arduino connection closed successfully");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("WarehouseInterface"),
                          "Exception while closing Arduino connection: " << e.what());
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"),
                          "Unknown error while closing connection with port " << port_);
    }
  }
}


CallbackReturn WarehouseInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Initializing WarehouseInterface...");
  
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("WarehouseInterface"), "Failed to initialize SystemInterface");
    return result;
  }

  // Log hardware info for debugging
  RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                     "Hardware name: " << info_.name);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                     "Number of joints: " << info_.joints.size());

  // Check for port parameter
  try
  {
    port_ = info_.hardware_parameters.at("port");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                       "Serial port configured: " << port_);
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("WarehouseInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // Validate joint configuration
  if (info_.joints.size() < 2)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                        "Expected at least 2 joints, got " << info_.joints.size());
    return CallbackReturn::FAILURE;
  }

  // Log joint names for debugging
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                       "Joint " << i << ": " << info_.joints[i].name);
  }

  // Size vectors to hold state for all joints
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();

  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "WarehouseInterface initialized successfully");
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> WarehouseInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Exporting state interfaces...");

  // Export state interfaces for all joints
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                       "Exported state interfaces for joint: " << info_.joints[i].name);
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                     "Total state interfaces exported: " << state_interfaces.size());
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> WarehouseInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Exporting command interfaces...");

  // Find wheel joints by name
  int left_wheel_idx = -1, right_wheel_idx = -1;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].name == "wheel_left_joint")
    {
      left_wheel_idx = i;
    }
    else if (info_.joints[i].name == "wheel_right_joint")
    {
      right_wheel_idx = i;
    }
  }

  if (left_wheel_idx == -1 || right_wheel_idx == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("WarehouseInterface"), 
                 "Could not find wheel joints. Left: %d, Right: %d", left_wheel_idx, right_wheel_idx);
    return command_interfaces;
  }

  // Provide velocity command interfaces for the wheel joints
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[left_wheel_idx].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[left_wheel_idx]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[right_wheel_idx].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[right_wheel_idx]));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                     "Exported velocity command interfaces for wheels");

  // Export the command interface for the goal trigger
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "goal_handler", "goal.goal_trigger", &goal_success_trigger_));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                     "Total command interfaces exported: " << command_interfaces.size());
  return command_interfaces;
}


CallbackReturn WarehouseInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Activating robot hardware ...");

  // Reset commands and states
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  goal_success_trigger_ = 0.0;

  // Check if serial port exists
  if (access(port_.c_str(), F_OK) != 0)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Serial port " << port_ << " does not exist!");
    return CallbackReturn::FAILURE;
  }

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WarehouseInterface"), 
                       "Opening serial port: " << port_);
    arduino_.Open(port_);
    
    RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Setting baud rate to 115200");
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    
    // Give Arduino time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), 
                "Serial connection established successfully");
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Exception while opening serial port " << port_ << ": " << e.what());
    return CallbackReturn::FAILURE;
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Unknown error while opening serial port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn WarehouseInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Deactivating robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
      RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Arduino connection closed");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("WarehouseInterface"),
                          "Exception while closing Arduino connection: " << e.what());
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("WarehouseInterface"),
                          "Unknown error while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("WarehouseInterface"), "Hardware deactivated");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type WarehouseInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  try
  {
    if(arduino_.IsDataAvailable())
    {
      auto dt = (rclcpp::Clock().now() - last_run_).seconds();
      std::string message;
      arduino_.ReadLine(message);
      
      if (!message.empty())
      {
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;
        
        while(std::getline(ss, res, ','))
        {
          if (res.length() < 3) continue;
          
          multiplier = res.at(1) == 'p' ? 1 : -1;

          if(res.at(0) == 'r')
          {
            velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
            position_states_.at(0) += velocity_states_.at(0) * dt;
          }
          else if(res.at(0) == 'l')
          {
            velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
            position_states_.at(1) += velocity_states_.at(1) * dt;
          }
        }
      }
      last_run_ = rclcpp::Clock().now();
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Exception in read(): " << e.what());
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type WarehouseInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  try
  {
    if (!arduino_.IsOpen())
    {
      RCLCPP_WARN(rclcpp::get_logger("WarehouseInterface"), "Arduino not connected, skipping write");
      return hardware_interface::return_type::OK;
    }

    // Ensure we have at least 2 velocity commands
    if (velocity_commands_.size() < 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("WarehouseInterface"), "Not enough velocity commands");
      return hardware_interface::return_type::ERROR;
    }

    // Build message string
    std::stringstream message_stream;
    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    
    std::string compensate_zeros_right = std::abs(velocity_commands_.at(0)) < 10.0 ? "0" : "";
    std::string compensate_zeros_left = std::abs(velocity_commands_.at(1)) < 10.0 ? "0" : "";

    // Check for goal success trigger and reset it
    bool goal_succeeded = false;
    if (goal_success_trigger_ == 1.0) {
        goal_succeeded = true;
        goal_success_trigger_ = 0.0;
    }
    
    message_stream << std::fixed << std::setprecision(2) << 
      "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
      ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << 
      ",g" << (goal_succeeded ? '1' : '0') << ",";

    arduino_.Write(message_stream.str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Exception in write(): " << e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("WarehouseInterface"),
                        "Unknown error in write()");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace warehouse_firmware

PLUGINLIB_EXPORT_CLASS(warehouse_firmware::WarehouseInterface, hardware_interface::SystemInterface)