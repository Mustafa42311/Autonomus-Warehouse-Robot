// ========================== Cytron Motor Driver (Non-PID) ==========================

// Cytron Motor Driver Pins
#define RIGHT_PWM 9         // Right motor PWM pin
#define RIGHT_DIR 12        // Right motor direction pin
#define LEFT_PWM 6          // Left motor PWM pin
#define LEFT_DIR 13         // Left motor direction pin

// Encoder Pins
#define RIGHT_ENCODER_PHASEA 3
#define RIGHT_ENCODER_PHASEB 5
#define LEFT_ENCODER_PHASEA 2
#define LEFT_ENCODER_PHASEB 7

// Encoder variables
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";
String left_wheel_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Command parsing
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;
double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

const double vel_to_pwm = 20.0; // Example scaling factor

void setup() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_DIR, LOW);

  Serial.begin(115200);

  pinMode(RIGHT_ENCODER_PHASEB, INPUT);
  pinMode(LEFT_ENCODER_PHASEB, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PHASEA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PHASEA), leftEncoderCallback, RISING);
}

void loop() {
  while (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p') {
      if (is_right_wheel_cmd) { digitalWrite(RIGHT_DIR, LOW); is_right_wheel_forward = true; }
      else if (is_left_wheel_cmd) { digitalWrite(LEFT_DIR, HIGH); is_left_wheel_forward = true; }
    }
    else if (chr == 'n') {
      if (is_right_wheel_cmd) { digitalWrite(RIGHT_DIR, HIGH); is_right_wheel_forward = false; }
      else if (is_left_wheel_cmd) { digitalWrite(LEFT_DIR, LOW); is_left_wheel_forward = false; }
    }
    else if (chr == ',') {
      value[value_idx] = '\0';
      if (is_right_wheel_cmd) right_wheel_cmd_vel = atof(value);
      if (is_left_wheel_cmd)  left_wheel_cmd_vel  = atof(value);
      value_idx = 0;
      memset(value, 0, sizeof(value));
    }
    else if ((chr >= '0' && chr <= '9') || chr == '.') {
      if (value_idx < 6) value[value_idx++] = chr;
    }
  }

  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_encoder_counter  * (60.0 / 385.0)) * 0.10472;

    right_wheel_cmd = right_wheel_cmd_vel * vel_to_pwm;
    left_wheel_cmd  = left_wheel_cmd_vel  * vel_to_pwm;

    right_wheel_cmd = constrain(right_wheel_cmd, 0, 255);
    left_wheel_cmd  = constrain(left_wheel_cmd, 0, 255);

    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel  == 0.0) left_wheel_cmd  = 0.0;
    
    //Send encoder readings to Raspberry Pi
    String encoder_read = "r" + String(right_wheel_sign) + String(abs(right_wheel_meas_vel), 2) + ",l" + String(left_wheel_sign) + String(abs(left_wheel_meas_vel), 2) + ",";
    Serial.println(encoder_read);

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(RIGHT_PWM, right_wheel_cmd);
    analogWrite(LEFT_PWM, left_wheel_cmd);
  }
}

void rightEncoderCallback() {
  if (digitalRead(RIGHT_ENCODER_PHASEB) == HIGH) right_wheel_sign = "p";
  else right_wheel_sign = "n";
  right_encoder_counter++;
}

void leftEncoderCallback() {
  if (digitalRead(LEFT_ENCODER_PHASEB) == HIGH) left_wheel_sign = "n";
  else left_wheel_sign = "p";
  left_encoder_counter++;
}