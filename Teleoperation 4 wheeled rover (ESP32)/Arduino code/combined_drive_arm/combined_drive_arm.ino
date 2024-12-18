#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Pins for the bot motors
#define BOT_LEFT_PWM_PIN 12
#define BOT_LEFT_DIR_PIN 13
#define BOT_RIGHT_PWM_PIN 14
#define BOT_RIGHT_DIR_PIN 27

// Pins for the robotic arm motors
#define ARM_MOTOR1_PWM_PIN 19
#define ARM_MOTOR1_DIR_PIN 21
#define ARM_MOTOR2_PWM_PIN 15
#define ARM_MOTOR2_DIR_PIN 4
#define ARM_MOTOR3_PWM_PIN 25
#define ARM_MOTOR3_DIR_PIN 26
#define ARM_MOTOR4_PWM_PIN 14
#define ARM_MOTOR4_DIR_PIN 27
#define ARM_MOTOR5_PWM_PIN 12
#define ARM_MOTOR5_DIR_PIN 13
#define ARM_STEPPER_DIR_PIN 4
#define ARM_STEPPER_STEP_PIN 16

// ROS 2 setup
rcl_subscription_t bot_subscriber;
rcl_subscription_t arm_subscriber;
geometry_msgs__msg__Twist bot_msg;
geometry_msgs__msg__Twist arm_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Macros for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Motor setup
void setupMotors() {
  // Bot motors
  pinMode(BOT_LEFT_PWM_PIN, OUTPUT);
  pinMode(BOT_LEFT_DIR_PIN, OUTPUT);
  pinMode(BOT_RIGHT_PWM_PIN, OUTPUT);
  pinMode(BOT_RIGHT_DIR_PIN, OUTPUT);

  // Robotic arm motors
  pinMode(ARM_MOTOR1_PWM_PIN, OUTPUT);
  pinMode(ARM_MOTOR1_DIR_PIN, OUTPUT);
  pinMode(ARM_MOTOR2_PWM_PIN, OUTPUT);
  pinMode(ARM_MOTOR2_DIR_PIN, OUTPUT);
  pinMode(ARM_MOTOR3_PWM_PIN, OUTPUT);
  pinMode(ARM_MOTOR3_DIR_PIN, OUTPUT);
  pinMode(ARM_MOTOR4_PWM_PIN, OUTPUT);
  pinMode(ARM_MOTOR4_DIR_PIN, OUTPUT);
  pinMode(ARM_MOTOR5_PWM_PIN, OUTPUT);
  pinMode(ARM_MOTOR5_DIR_PIN, OUTPUT);
  pinMode(ARM_STEPPER_DIR_PIN, OUTPUT);
  pinMode(ARM_STEPPER_STEP_PIN, OUTPUT);

  // Configure PWM channels for ESP32
  ledcSetup(0, 5000, 8); // Channel 0 for bot left motor
  ledcSetup(1, 5000, 8); // Channel 1 for bot right motor
  ledcSetup(2, 5000, 8); // Channel 2 for arm motor 1
  ledcSetup(3, 5000, 8); // Channel 3 for arm motor 2
  ledcSetup(4, 5000, 8); // Channel 4 for arm motor 3
  ledcSetup(5, 5000, 8); // Channel 5 for arm motor 4
  ledcSetup(6, 5000, 8); // Channel 6 for arm motor 5

  ledcAttachPin(BOT_LEFT_PWM_PIN, 0);
  ledcAttachPin(BOT_RIGHT_PWM_PIN, 1);
  ledcAttachPin(ARM_MOTOR1_PWM_PIN, 2);
  ledcAttachPin(ARM_MOTOR2_PWM_PIN, 3);
  ledcAttachPin(ARM_MOTOR3_PWM_PIN, 4);
  ledcAttachPin(ARM_MOTOR4_PWM_PIN, 5);
  ledcAttachPin(ARM_MOTOR5_PWM_PIN, 6);
}

// Bot motor control
void controlBotMotors(float left_velocity, float right_velocity) {
  int left_speed = (int)(255 * constrain(left_velocity, -1.0, 1.0));
  int right_speed = (int)(255 * constrain(right_velocity, -1.0, 1.0));

  if (left_speed > 0) {
    digitalWrite(BOT_LEFT_DIR_PIN, HIGH);
  } else {
    digitalWrite(BOT_LEFT_DIR_PIN, LOW);
    left_speed = -left_speed;
  }
  ledcWrite(0, left_speed);

  if (right_speed > 0) {
    digitalWrite(BOT_RIGHT_DIR_PIN, HIGH);
  } else {
    digitalWrite(BOT_RIGHT_DIR_PIN, LOW);
    right_speed = -right_speed;
  }
  ledcWrite(1, right_speed);
}

// Robotic arm motor control
void controlArmMotors(const geometry_msgs__msg__Twist *cmd) {
  int speed = 200; // Fixed speed (half of 255)
  bool dir = cmd->angular.z > 0.1 ? HIGH : (cmd->angular.z < -0.1 ? LOW : -1);

  if (dir != -1) {
    digitalWrite(MOTOR1_DIR_PIN, dir);
    digitalWrite(MOTOR2_DIR_PIN, dir);
    digitalWrite(MOTOR3_DIR_PIN, dir);
    digitalWrite(MOTOR4_DIR_PIN, dir);
    digitalWrite(MOTOR5_DIR_PIN, dir);
    digitalWrite(STEPPER_DIR_PIN, dir);
  }

  ledcWrite(0, cmd->linear.x * speed);
  ledcWrite(1, cmd->linear.y * speed);
  ledcWrite(2, cmd->linear.z * speed);
  ledcWrite(3, cmd->angular.x * speed);
  ledcWrite(4, cmd->angular.y * speed);
}

// Callback for bot topic
void bot_subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  controlBotMotors(msg->linear.x, msg->angular.z);
}

// Callback for arm topic
void arm_subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  controlArmMotors(msg);
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "combined_esp32_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &bot_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(
      &arm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_1"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &bot_subscriber, &bot_msg, &bot_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_subscriber, &arm_msg, &arm_subscription_callback, ON_NEW_DATA));

  setupMotors();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
