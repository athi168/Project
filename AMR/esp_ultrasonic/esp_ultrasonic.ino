#include <HCSR04.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

HCSR04 hc_front(27, 25);  // (trig pin = GPIO27, echo pin = GPIO25)
HCSR04 hc_left(27, 0);   // (trig pin = GPIO27, echo pin = GPIO0)
HCSR04 hc_right(27, 23);  // (trig pin = GPIO27, echo pin = GPIO23)
HCSR04 hc_rear(27, 5);   // (trig pin = GPIO27, echo pin = GPIO5)

rcl_publisher_t ultrasonic_publisher_front;
rcl_publisher_t ultrasonic_publisher_left;
rcl_publisher_t ultrasonic_publisher_right;
rcl_publisher_t ultrasonic_publisher_rear;

std_msgs__msg__Int32 ultrasonic_msg_front;
std_msgs__msg__Int32 ultrasonic_msg_left;
std_msgs__msg__Int32 ultrasonic_msg_right;
std_msgs__msg__Int32 ultrasonic_msg_rear;

rclc_executor_t executor; //******************************* */

void setup() {
  Serial.begin(115200);  // Baudrate 115200
  Serial.println("ESP32 - RCWL-1655/JSN-SR04T Trig-Echo Test");

  // Initialize micro-ROS
  set_microros_transports();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "ultrasonic_node", "", &support);

  rclc_publisher_init_default(
    &ultrasonic_publisher_front,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/ultrasonic/distance_front");

  rclc_publisher_init_default(
    &ultrasonic_publisher_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/ultrasonic/distance_left");
  
  rclc_publisher_init_default(
    &ultrasonic_publisher_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/ultrasonic/distance_right");

  rclc_publisher_init_default(
    &ultrasonic_publisher_rear,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/ultrasonic/distance_rear");

    rclc_executor_init(&executor, &support.context, 1, &allocator); //******************************* */

}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); //******************************* */
  float distance_front = hc_front.dist();  // Đọc khoảng cách trước (cm)
  float distance_left = hc_left.dist();    // Đọc khoảng cách trái (cm)
  float distance_right = hc_right.dist();  // Đọc khoảng cách phải (cm)
  float distance_rear = hc_rear.dist();    // Đọc khoảng cách sau (cm)

  Serial.print("Front: "); Serial.print(distance_front);
  Serial.print(" | Left: "); Serial.print(distance_left);
  Serial.print(" | Right: "); Serial.print(distance_right);
  Serial.print(" | Rear: "); Serial.println(distance_rear);
  
  // Publish distance_front to /ultrasonic/distance_front
  ultrasonic_msg_front.data = static_cast<int32_t>(distance_front);
  rcl_publish(&ultrasonic_publisher_front, &ultrasonic_msg_front, NULL);

  // Publish distance_left to /ultrasonic/distance_left
  ultrasonic_msg_left.data = static_cast<int32_t>(distance_left);
  rcl_publish(&ultrasonic_publisher_left, &ultrasonic_msg_left, NULL);

  // Publish distance_right to /ultrasonic/distance_right
  ultrasonic_msg_right.data = static_cast<int32_t>(distance_right);
  rcl_publish(&ultrasonic_publisher_right, &ultrasonic_msg_right, NULL);

  // Publish distance_rear to /ultrasonic/distance_rear
  ultrasonic_msg_rear.data = static_cast<int32_t>(distance_rear);
  rcl_publish(&ultrasonic_publisher_rear, &ultrasonic_msg_rear, NULL);

  delay(100);  // Delay 100ms
}
