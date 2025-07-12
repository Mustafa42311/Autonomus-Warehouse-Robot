#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "warehouse_msgs/action/warehouse_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


using namespace std::placeholders;

namespace warehouse_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<warehouse_msgs::action::WarehouseTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<warehouse_msgs::action::WarehouseTask>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> rail_move_group_, clamp_hand_move_group_;
  std::vector<double> rail_joint_goal_,clamp_hand_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const warehouse_msgs::action::WarehouseTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_msgs::action::WarehouseTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(rail_move_group_){
      rail_move_group_->stop();
    }

    if(clamp_hand_move_group_){
      clamp_hand_move_group_->stop();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_msgs::action::WarehouseTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<warehouse_msgs::action::WarehouseTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<warehouse_msgs::action::WarehouseTask::Result>();

    // MoveIt 2 Interface
   if(!rail_move_group_){
      rail_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "rail");
    }
    if(!clamp_hand_move_group_){
      clamp_hand_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "clamp_hand");
    }


    // CORRECTED GOAL PARAMETERS
    if (goal_handle->get_goal()->task_number == 0) // Represents a "Home" or fully open/retracted state
    {
      // Set goals to the minimum limit for each joint.
      // Since rack joints mimic the gear joint, their goals should be consistent.
      rail_joint_goal_ = {-0.01};
      clamp_hand_joint_goal_ = {0.0, 0.0, 0.0}; 
      // The rail group only has one joint, so its goal vector must have only one value.
      
    }
    else if (goal_handle->get_goal()->task_number == 1) // Represents an "Intermediate" state
    {
      // Set goals to a midpoint within the valid range.
      rail_joint_goal_ = {0.15};
      clamp_hand_joint_goal_ = {0.05, 0.05, 0.05};
      
    }
    else if (goal_handle->get_goal()->task_number == 2) // Represents a "Closed" or fully extended state
    {
      // Set goals to the maximum limit for each joint.
      rail_joint_goal_ = {0.3};
      clamp_hand_joint_goal_ = {0.1, 0.1, 0.1};
      
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      result->success = false;
      goal_handle->abort(result); // Abort the goal if the task number is invalid
      return;
    }
    
    rail_move_group_->setStartStateToCurrentState();
    clamp_hand_move_group_->setStartStateToCurrentState();
    
    bool rail_within_bounds = rail_move_group_->setJointValueTarget(rail_joint_goal_);
    bool clamp_hand_within_bounds = clamp_hand_move_group_->setJointValueTarget(clamp_hand_joint_goal_);
    
    if (!rail_within_bounds || !clamp_hand_within_bounds) 
    {
      RCLCPP_WARN(get_logger(),
                  "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan rail_plan;
    moveit::planning_interface::MoveGroupInterface::Plan clamp_hand_plan;
    
    bool rail_plan_success = (rail_move_group_->plan(rail_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool clamp_hand_plan_success = (clamp_hand_move_group_->plan(clamp_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    
    if(rail_plan_success && clamp_hand_plan_success) 
    {
      // UPDATED LOG MESSAGE TO REFLECT NEW ORDER
      RCLCPP_INFO(get_logger(), "Planner SUCCEEDED, moving the rail and the clamp_hand sequentially.");

      // Execute the rail plan first
      moveit::core::MoveItErrorCode rail_result = rail_move_group_->execute(rail_plan);

      // If the rail movement was successful, execute the clamp hand plan
      if (rail_result == moveit::core::MoveItErrorCode::SUCCESS) {
        moveit::core::MoveItErrorCode hand_result = clamp_hand_move_group_->execute(clamp_hand_plan);
        if (hand_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Clamp hand movement failed!");
            result->success = false;
            goal_handle->abort(result);
            return;
        }
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Rail movement failed!");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
  
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};
}  // namespace warehouse_remote

RCLCPP_COMPONENTS_REGISTER_NODE(warehouse_remote::TaskServer)