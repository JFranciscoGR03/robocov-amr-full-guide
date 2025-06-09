#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

#include <VescUart.h>
#include <HardwareSerial.h>

// UARTs para VESC
HardwareSerial VESCSerial1(2);  // RX=16, TX=17 -> RUEDA IZQUIERDA
HardwareSerial VESCSerial2(1);  // RX=23, TX=22 -> RUEDA DERECHA

VescUart vesc1;
VescUart vesc2;

// ROS
rcl_subscription_t subscriber;
rcl_publisher_t vel_left_publisher;
rcl_publisher_t vel_right_publisher;
rcl_timer_t velocity_timer;

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 vel_left_msg;
std_msgs__msg__Float32 vel_right_msg;

#define LED_BUILTIN 2

// Robot físico
const float wheel_radius = 0.08;         // 8 cm
const float wheel_separation = 0.6 / 2;  // 60 cm entre ruedas
const int motor_poles = 16;

float velocityEncL = 0.0;
float velocityEncR = 0.0;

bool publish_left_next = true;  // Alternancia

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// Callback que alterna la publicación entre ruedas
void publish_encoder_velocity_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  if (publish_left_next) {
    if (vesc1.getVescValues()) {
      velocityEncL = (vesc1.data.rpm / motor_poles) / (60.0 / (2 * PI));
      vel_left_msg.data = velocityEncL;
      RCSOFTCHECK(rcl_publish(&vel_left_publisher, &vel_left_msg, NULL));
    }
  } else {
    if (vesc2.getVescValues()) {
      velocityEncR = (vesc2.data.rpm / motor_poles) / (60.0 / (2 * PI));
      vel_right_msg.data = velocityEncR;
      RCSOFTCHECK(rcl_publish(&vel_right_publisher, &vel_right_msg, NULL));
    }
  }

  publish_left_next = !publish_left_next;  // alterna
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

  float v = msg->linear.x;
  float w = msg->angular.z;

  float w_left = (v / wheel_radius) - (w * (wheel_separation / wheel_radius));
  float w_right = (v / wheel_radius) + (w * (wheel_separation / wheel_radius));

  float rpm_left = (60.0 / (2 * PI)) * w_left;
  float rpm_right = (60.0 / (2 * PI)) * w_right;

  float erpm_left = rpm_left * motor_poles * -1;
  float erpm_right = rpm_right * motor_poles * -1;

  vesc1.setRPM((int)erpm_left);
  vesc2.setRPM((int)erpm_right);
}

void setup() {
  VESCSerial1.begin(115200, SERIAL_8N1, 16, 17);
  VESCSerial2.begin(115200, SERIAL_8N1, 23, 22);
  vesc1.setSerialPort(&VESCSerial1);
  vesc2.setSerialPort(&VESCSerial2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // micro-ROS setup
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "vesc_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &vel_left_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocityEncL"));

  RCCHECK(rclc_publisher_init_default(
    &vel_right_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocityEncR"));

  RCCHECK(rclc_timer_init_default(
    &velocity_timer,
    &support,
    RCL_MS_TO_NS(100),
    publish_encoder_velocity_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &velocity_timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}