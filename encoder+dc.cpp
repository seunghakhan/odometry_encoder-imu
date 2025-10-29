/**
 * motor_encoder_driver.ino  (ESP32 전용)
 *  - DC 모터 PWM + 방향 제어  (핀: DIR=4, PWM=5)
 *  - 쿼드러처 엔코더 카운트   (핀: A=32, B=33)
 *  - micro-ROS
 *      /motor_cmd      (std_msgs/Float32)  ← 속도 명령  [-255,255]
 *      /encoder_count  (std_msgs/Int32)    → 누적 틱     10 Hz
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <cmath>   // fabs

/* ───── 인코더 핀 & ISR ───── */
const int encoderPinA = 32;   // 입력 전용 GPIO
const int encoderPinB = 33;

volatile long encoderPos  = 0;
volatile int  lastEncoded = 0;

void IRAM_ATTR updateEncoder()
{
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int sum     = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;

  lastEncoded = encoded;
}

/* ───── 모터 핀 ───── */
#define MOTOR_PWM_PIN 5   // LEDC 채널 0 기본
#define MOTOR_DIR_PIN 4

/* ───── micro-ROS 리소스 ───── */
rcl_node_t          node;
rcl_subscription_t  motor_cmd_sub;
rcl_publisher_t     enc_pub;
rclc_executor_t     executor;

std_msgs__msg__Float32 motor_cmd_msg;
std_msgs__msg__Int32   enc_msg;

/* ───── /motor_cmd 콜백 ───── */
void motor_cmd_callback(const void * msgin)
{
  const auto * msg = (const std_msgs__msg__Float32 *)msgin;
  float cmd = msg->data;                       // -255 ~ 255
  float duty = fabs(cmd);
  duty = constrain(duty, 0.0f, 255.0f);        // 8-bit PWM 범위

  digitalWrite(MOTOR_DIR_PIN, (cmd >= 0.0f));  // 정/역
  analogWrite(MOTOR_PWM_PIN, (uint8_t)duty);   // 0-255
}

/* ───── Arduino setup ───── */
void setup()
{
  /* ① USB-Serial → micro-ROS transport */
  Serial.begin(115200);
  delay(1000);
  set_microros_transports();   // 반드시 begin 이후 호출

  /* ② 엔코더 핀 & 인터럽트 */
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  /* ③ 모터 제어 핀 */
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  /* ④ micro-ROS 객체 초기화 */
  rcl_allocator_t alloc;
  rclc_support_t  support;
  alloc = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &alloc);

  rclc_node_init_default(&node, "motor_encoder_node", "", &support);

  /* /motor_cmd 구독 */
  rclc_subscription_init_default(
      &motor_cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/motor_cmd");

  /* /encoder_count 퍼블리셔 */
  rclc_publisher_init_default(
      &enc_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/encoder_count");

  /* executor : 콜백 1개(구독)만 처리 */
  rclc_executor_init(&executor, &support.context, 1, &alloc);
  rclc_executor_add_subscription(
      &executor, &motor_cmd_sub, &motor_cmd_msg,
      &motor_cmd_callback, ON_NEW_DATA);
}

/* ───── Arduino loop ───── */
void loop()
{
  /* 1) micro-ROS 콜백 처리 (20 Hz) */
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

  /* 2) 100 ms 주기로 엔코더 값 퍼블리시 */
  static uint32_t last_pub = 0;
  if (millis() - last_pub >= 100) {
    last_pub = millis();
    enc_msg.data = encoderPos;
    rcl_publish(&enc_pub, &enc_msg, NULL);
  }
}
