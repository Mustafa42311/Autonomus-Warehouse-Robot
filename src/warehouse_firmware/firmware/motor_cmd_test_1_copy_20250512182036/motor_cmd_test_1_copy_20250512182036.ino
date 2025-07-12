#include <PID_v1.h>

// Motor Driver Pins
#define PWM_A 3
#define PWM_B 6
#define DIR_B 7
#define DIR_A 4

// Encoder input pins
#define right_encoder_phaseA 8
#define right_encoder_phaseB 9
#define left_encoder_phaseA 10
#define left_encoder_phaseB 11

unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[7] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;
double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

unsigned long last_millis = 0;
const unsigned long interval = 100;

void setup() {
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // SERIAL COMMAND PARSING
  while (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true; is_left_wheel_cmd = false; value_idx = 0;
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false; is_left_wheel_cmd = true; value_idx = 0;
    }
    else if (chr == 'p') {
      if (is_right_wheel_cmd) { digitalWrite(DIR_A, LOW); is_right_wheel_forward = true; }
      else if (is_left_wheel_cmd) { digitalWrite(DIR_B, LOW); is_left_wheel_forward = true; }
    }
    else if (chr == 'n') {
      if (is_right_wheel_cmd) { digitalWrite(DIR_A, HIGH); is_right_wheel_forward = false; }
      else if (is_left_wheel_cmd) { digitalWrite(DIR_B, HIGH); is_left_wheel_forward = false; }
    }
    else if (chr == ',') {
      value[value_idx] = '\0';
      if (is_right_wheel_cmd) right_wheel_cmd_vel = atof(value);
      if (is_left_wheel_cmd)  left_wheel_cmd_vel  = atof(value);
      value_idx = 0; memset(value, 0, sizeof(value));
    }
    else if ((chr >= '0' && chr <= '9') || chr == '.') {
      if (value_idx < 6) value[value_idx++] = chr;
    }
  }

  // MAIN CONTROL LOOP
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_encoder_counter  * (60.0 / 385.0)) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();

    // Clamp commands to valid PWM range
    right_wheel_cmd = constrain(right_wheel_cmd, 0, 255);
    left_wheel_cmd  = constrain(left_wheel_cmd, 0, 255);

    // If commanded velocity is zero, stop motor
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel  == 0.0) left_wheel_cmd  = 0.0;

    // Send velocity feedback to ROS
    Serial.print("rp");
    if (right_wheel_meas_vel < 10.0) Serial.print("0");
    Serial.print(abs(right_wheel_meas_vel), 2);
    Serial.print(",ln");
    if (left_wheel_meas_vel < 10.0) Serial.print("0");
    Serial.print(abs(left_wheel_meas_vel), 2);
    Serial.println(",");

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(PWM_A, right_wheel_cmd);
    analogWrite(PWM_B, left_wheel_cmd);
  }
}

void rightEncoderCallback() {
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_encoder_counter++;
}
