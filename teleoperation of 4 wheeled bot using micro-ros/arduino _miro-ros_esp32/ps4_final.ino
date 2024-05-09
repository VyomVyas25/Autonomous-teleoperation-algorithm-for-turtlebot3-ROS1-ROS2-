// #include <micro_ros_arduino.h>
// #include <std_msgs/msg/float64.h>

// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #define LED_PIN 13
// // Define motor A pins
// #define MOTOR_A1_PIN 2
// #define MOTOR_A2_PIN 3
// #define MOTOR_A_EN_PIN 5 // PWM pin for speed control

// // Define motor B pins
// #define MOTOR_B1_PIN 4
// #define MOTOR_B2_PIN 6
// #define MOTOR_B_EN_PIN 9 // PWM pin for speed control


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// const float MAX_SPEED = 100.0; // Adjust as needed

// // Define global variables for motor speeds
// float left_motor_speed = 0.0;
// float right_motor_speed = 0.0;

// rcl_subscription_t subscriber;
// std_msgs__msg__Float64 ps4_data;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// void error_loop(){
//   while(1){
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
// }

// // Define callback function to process PS4 controller data
// void ps4ControllerCallback(const void * msgin){
//   const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  
//   // Adjust motor speeds based on PS4 controller data
//   float speed_factor = msg->data; // Value from -1 to 1
  
//   // Adjust speed range and direction based on axes
//   left_motor_speed = MAX_SPEED * speed_factor;
//   right_motor_speed = MAX_SPEED * speed_factor;

//   // Print velocity data
//   Serial.print("Left Motor Speed: ");
//   Serial.println(left_motor_speed);
//   Serial.print("Right Motor Speed: ");
//   Serial.println(right_motor_speed);
// }

// void setup() {
//   set_microros_transports();

//   // Initialize serial communication
//   Serial.begin(9600);
  
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, HIGH);  
  
//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   // Create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // Create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

//   // Create subscription
//   RCCHECK(rclc_subscription_init_default(
//     &subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
//     "/ps4_controller_data")); // Update topic name as needed

//   // Create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

//   // Add subscription to the executor with callback function
//   RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &ps4_data, &ps4ControllerCallback, ON_NEW_DATA));
// }

// void loop() {
//   delay(100);
  
//   // Control left side motors
//   // Assuming Motor A1 and Motor A2 are connected to Motor Driver 1
//   // Adjust pin numbers and logic levels according to your motor driver specifications
//   digitalWrite(MOTOR_A1_PIN, left_motor_speed > 0 ? HIGH : LOW);
//   digitalWrite(MOTOR_A2_PIN, left_motor_speed < 0 ? HIGH : LOW);
//   analogWrite(MOTOR_A_EN_PIN, abs(left_motor_speed)); // Adjust PWM pin according to your motor driver

//   // Control right side motors
//   // Assuming Motor B1 and Motor B2 are connected to Motor Driver 2
//   // Adjust pin numbers and logic levels according to your motor driver specifications
//   digitalWrite(MOTOR_B1_PIN, right_motor_speed > 0 ? HIGH : LOW);
//   digitalWrite(MOTOR_B2_PIN, right_motor_speed < 0 ? HIGH : LOW);
//   analogWrite(MOTOR_B_EN_PIN, abs(right_motor_speed)); // Adjust PWM pin according to your motor driver
// }
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float64.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define LED_PIN 13
// Define motor A pins
#define MOTOR_A1_PIN 2
#define MOTOR_A2_PIN 3
#define MOTOR_A_EN_PIN 5 // PWM pin for speed control

// Define motor B pins
#define MOTOR_B1_PIN 4
#define MOTOR_B2_PIN 6
#define MOTOR_B_EN_PIN 9 // PWM pin for speed control

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const float MAX_SPEED = 100.0; // Adjust as needed

// Define global variables for motor speeds
float left_motor_speed = 0.0;
float right_motor_speed = 0.0;

rcl_subscription_t subscriber;
std_msgs__msg__Float64 ps4_data;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Define callback function to process PS4 controller data
void ps4ControllerCallback(const void * msgin){
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  
  // Adjust motor speeds based on PS4 controller data
  float speed_factor = msg->data; // Value from -1 to 1
  
  // Adjust speed range and direction based on axes
  left_motor_speed = MAX_SPEED * speed_factor;
  right_motor_speed = MAX_SPEED * speed_factor;

  // Print the motor speeds to the terminal
  Serial.print("Left motor speed: ");
  Serial.println(left_motor_speed);
  Serial.print("Right motor speed: ");
  Serial.println(right_motor_speed);
}


void setup() {
  set_microros_transports();
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/ps4_controller_data")); // Update topic name as needed

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Add subscription to the executor with callback function
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &ps4_data, &ps4ControllerCallback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  
  // Control left side motors
  // Assuming Motor A1 and Motor A2 are connected to Motor Driver 1
  // Adjust pin numbers and logic levels according to your motor driver specifications
  digitalWrite(MOTOR_A1_PIN, left_motor_speed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_A2_PIN, left_motor_speed < 0 ? HIGH : LOW);
  analogWrite(MOTOR_A_EN_PIN, abs(left_motor_speed)); // Adjust PWM pin according to your motor driver

  // Control right side motors
  // Assuming Motor B1 and Motor B2 are connected to Motor Driver 2
  // Adjust pin numbers and logic levels according to your motor driver specifications
  digitalWrite(MOTOR_B1_PIN, right_motor_speed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_B2_PIN, right_motor_speed < 0 ? HIGH : LOW);
  analogWrite(MOTOR_B_EN_PIN, abs(right_motor_speed)); // Adjust PWM pin according to your motor driver
}
