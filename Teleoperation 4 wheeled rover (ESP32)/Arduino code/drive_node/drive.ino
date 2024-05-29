#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#define MOTOR_LEFT_PWM_PIN 13
#define MOTOR_LEFT_DIR_PIN 12
#define MOTOR_RIGHT_PWM_PIN 14
#define MOTOR_RIGHT_DIR_PIN 26

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setupMotors() {
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

  // Configure PWM channels for ESP32
  ledcSetup(0, 5000, 8); // Channel 0, 5 kHz PWM, 8-bit resolution
  ledcSetup(1, 5000, 8); // Channel 1, 5 kHz PWM, 8-bit resolution

  ledcAttachPin(MOTOR_LEFT_PWM_PIN, 0); 
  ledcAttachPin(MOTOR_RIGHT_PWM_PIN, 1); 
}

void controlMotors(float left_velocity, float right_velocity) {
  int left_speed = (int)(255 * constrain(left_velocity, -1.0, 1.0));
  int right_speed = (int)(255 * constrain(right_velocity, -1.0, 1.0));

  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
    left_speed = -left_speed;
  }
  ledcWrite(0, left_speed);

  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
    right_speed = -right_speed;
  }
  ledcWrite(1, right_speed);
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float left_velocity = msg->linear.x;
  float right_velocity = msg->angular.z;

  controlMotors(left_velocity, right_velocity);
}

void setup() {
  Serial.begin(115200); 
  set_microros_transports();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  setupMotors();
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); // Reduced delay for faster response
}