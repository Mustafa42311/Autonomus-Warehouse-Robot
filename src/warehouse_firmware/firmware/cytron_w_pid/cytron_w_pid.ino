#include <PID_v1.h>

// --- Cytron Motor Driver Connection PINs ---
// Note: A Cytron driver uses 1 PWM pin for speed and 1 DIR pin for direction.
// We are using some of the original pin numbers to minimize wiring changes.
#define RIGHT_MOTOR_PWM 9   // PWM for Right Motor Speed (was L298N_enA)
#define RIGHT_MOTOR_DIR 12  // DIR for Right Motor Direction (was L298N_in1)
#define LEFT_MOTOR_PWM  6  // PWM for Left Motor Speed (was L298N_enB)
#define LEFT_MOTOR_DIR  13   // DIR for Left Motor Direction (was L298N_in4)

// Define motor direction logic for clarity.
// If your motors spin the wrong way, you can swap HIGH and LOW here.
#define FORWARD HIGH
#define REVERSE LOW

// --- Wheel Encoders Connection PINs (Unchanged) ---
#define right_encoder_phaseA 3  // Interrupt
#define right_encoder_phaseB 5
#define left_encoder_phaseA  2  // Interrupt
#define left_encoder_phaseB  4

// --- Encoders (Unchanged) ---
volatile unsigned int right_encoder_counter = 0; // Made volatile for safe use in ISR
volatile unsigned int left_encoder_counter = 0;  // Made volatile for safe use in ISR
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";   // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// --- Interpret Serial Messages (Unchanged) ---
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
// These booleans track the current direction sent to the driver
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// --- PID (Unchanged) ---
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;         // PWM value (0-255)
double left_wheel_cmd = 0.0;          // PWM value (0-255)
// Tuning Parameters
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 11.5;
double Ki_l = 7.5;
double Kd_l = 0.1;
// PID Controller Instances
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // --- Initialize Cytron Driver Pins ---
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);

  // Set initial motor direction to FORWARD
  digitalWrite(RIGHT_MOTOR_DIR, FORWARD);
  digitalWrite(LEFT_MOTOR_DIR, FORWARD);

  // Start PID controllers
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  // Start Serial communication
  Serial.begin(115200);

  // --- Initialize Encoders (Unchanged) ---
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // --- Read and Interpret Wheel Velocity Commands from Serial ---
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if (chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if (chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // --- DIRECTION CONTROL (MODIFIED FOR CYTRON) ---
    // Positive/Forward direction
    else if (chr == 'p')
    {
      if (is_right_wheel_cmd) {
        digitalWrite(RIGHT_MOTOR_DIR, FORWARD);
        is_right_wheel_forward = true;
      }
      else if (is_left_wheel_cmd) {
        digitalWrite(LEFT_MOTOR_DIR, FORWARD);
        is_left_wheel_forward = true;
      }
    }
    // Negative/Reverse direction
    else if (chr == 'n')
    {
      if (is_right_wheel_cmd) {
        digitalWrite(RIGHT_MOTOR_DIR, REVERSE);
        is_right_wheel_forward = false;
      }
      else if (is_left_wheel_cmd) {
        digitalWrite(LEFT_MOTOR_DIR, REVERSE);
        is_left_wheel_forward = false;
      }
    }
    // Separator (logic unchanged)
    else if (chr == ',')
    {
      if (is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if (is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value (logic unchanged)
    else
    {
      if (value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // --- PID Control Loop (runs every 'interval' milliseconds) ---
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval)
  {
    // Temporarily disable interrupts to safely read volatile encoder counts
    noInterrupts();
    unsigned int right_counts = right_encoder_counter;
    unsigned int left_counts = left_encoder_counter;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    // Calculate measured velocity in rad/s
    // Formula: (pulses/sec) * (1 rev / 385 pulses) * (60 sec / 1 min) = RPM
    //          RPM * (2*PI rad / 60 sec) = rad/s
    // Simplified: ( (counts / interval_sec) / PPR ) * 2*PI
    // Here: (counts / 0.1s) / 385 * 2 * 3.14159 = counts * 0.01632
    // The original formula is also correct:
    // (10 * counts * (60.0/385.0)) is RPM. RPM * 0.10472 (which is 2*PI/60) is rad/s.
    right_wheel_meas_vel = (10 * right_counts * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_counts * (60.0 / 385.0)) * 0.10472;

    // Compute PID output (PWM value)
    rightMotor.Compute();
    leftMotor.Compute();

    // Stop motors if the commanded velocity is zero
    if (right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if (left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    // Send encoder velocity readings back over serial
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    // Update timer
    last_millis = current_millis;

    // --- SPEED CONTROL (MODIFIED FOR CYTRON) ---
    // Send the PID output as a PWM signal to the Cytron drivers
    analogWrite(RIGHT_MOTOR_PWM, right_wheel_cmd);
    analogWrite(LEFT_MOTOR_PWM, left_wheel_cmd);
  }
}

// --- INTERRUPT SERVICE ROUTINES (Unchanged) ---

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  // Read phase B to determine direction of rotation
  if (digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "p"; // Forward
  }
  else
  {
    right_wheel_sign = "n"; // Reverse
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  // Read phase B to determine direction of rotation
  // Note: Left wheel direction may be inverted depending on wiring/mounting
  if (digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "n"; // Reverse
  }
  else
  {
    left_wheel_sign = "p"; // Forward
  }
  left_encoder_counter++;
}