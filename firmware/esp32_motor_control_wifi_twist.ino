#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// WiFi Settings
 char* ssid = "iPhone de Achraf";
 char* password = "12345678m";
 char* agent_ip = "172.20.10.10";  // PC's IP
const uint16_t agent_port = 8888;

// Motor Pins
#define SPEED_PIN 25
#define STEER_PIN 26

// Micro-ROS Objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;

// Motor control
void set_motors(float left, float right) {
  dacWrite(SPEED_PIN, left);
  dacWrite(STEER_PIN, right);
}

void twist_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  
  // Convert m/s to PWM (adjust these values as needed)
  float linear = msg->linear.x ;
  float angular = msg->angular.z ;
  linear=constrain(linear,-10,10);
    angular=constrain(angular,-10,10);

  
  // Differential drive
  float speed = map(linear,-10,10,50,205);
  float steer = map(angular,-10,10,50,205);

  
  set_motors(speed, steer);
  
  Serial.printf("CMD: speed=%.1f steer=%.1f\n", speed, steer);
}

void setup() {
  Serial.begin(115200);
  pinMode(SPEED_PIN, OUTPUT);
  pinMode(STEER_PIN, OUTPUT);
  set_motors(0, 0);  // Stop motors

  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Micro-ROS Setup
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_controller", "", &support);
  
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
  
  Serial.println("Ready for Twist commands!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  } else {
    set_motors(0, 0);  // Emergency stop
    delay(1000);
  }
}