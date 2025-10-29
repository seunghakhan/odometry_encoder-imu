/**
 * steer_imu_driver_6250.ino
 *  ESP32 + MPU6250 (MPU6050 호환)  |  micro-ROS
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>

#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050_light.h>  // ✅ MPU6250 호환 라이브러리

#define SERVO_PIN 25
Servo steer_servo;
MPU6050 mpu(Wire);  // ✅ MPU6250

rcl_node_t node;
rcl_subscription_t steer_sub;
rcl_publisher_t imu_pub;
rclc_executor_t executor;

std_msgs__msg__Float32 steer_msg;
sensor_msgs__msg__Imu  imu_msg;

/* ----- 서보 콜백 ----- */
void steer_callback(const void * msgin)
{
  auto * msg = (const std_msgs__msg__Float32 *)msgin;
  int pwm = constrain(90 + int(msg->data), 0, 180);
  steer_servo.write(pwm);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  set_microros_transports();

  /* IMU 초기화 */
  Wire.begin(21, 22);
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("❌ IMU 초기화 실패");
    while (1);
  }
  mpu.calcGyroOffsets();  // ✅ 자이로 정지 보정

  /* 서보 초기화 */
  steer_servo.setPeriodHertz(50);
  steer_servo.attach(SERVO_PIN, 500, 2500);

  /* micro-ROS 초기화 */
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t  support;
  rclc_support_init(&support, 0, NULL, &alloc);

  rclc_node_init_default(&node, "steer_imu_node", "", &support);

  rclc_subscription_init_default(
      &steer_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/steering_motor_cmd");

  rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw");

  rclc_executor_init(&executor, &support.context, 1, &alloc);
  rclc_executor_add_subscription(
      &executor, &steer_sub, &steer_msg,
      &steer_callback, ON_NEW_DATA);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));  // 20Hz

  /* 10Hz IMU 퍼블리시 */
  static uint32_t last = 0;
  if (millis() - last >= 100) {
    last = millis();
    mpu.update();

    uint64_t t = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec     = t / 1000000000ULL;
    imu_msg.header.stamp.nanosec = t % 1000000000ULL;
    imu_msg.header.frame_id.data = (char *)"imu_link";
    imu_msg.header.frame_id.size = 8;
    imu_msg.header.frame_id.capacity = 8;

    const float G_TO_MS2 = 9.80665f;
    const float DEG2RAD  = M_PI / 180.0f;

    imu_msg.linear_acceleration.x = mpu.getAccX() * G_TO_MS2;
    imu_msg.linear_acceleration.y = mpu.getAccY() * G_TO_MS2;
    imu_msg.linear_acceleration.z = mpu.getAccZ() * G_TO_MS2;

    imu_msg.angular_velocity.x = mpu.getGyroX() * DEG2RAD;
    imu_msg.angular_velocity.y = mpu.getGyroY() * DEG2RAD;
    imu_msg.angular_velocity.z = mpu.getGyroZ() * DEG2RAD;

    imu_msg.orientation_covariance[0] = -1.0;

    rcl_publish(&imu_pub, &imu_msg, NULL);
  }
}
