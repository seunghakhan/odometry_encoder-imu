#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

class SensorPostProcess : public rclcpp::Node
{
public:
  SensorPostProcess()
  : Node("sensor_postprocess"), enc_offset_(0), first_enc_(true), has_enc_(false), has_imu_(false)
  {
    // 구독자 생성
    sub_enc_ = create_subscription<std_msgs::msg::Int32>(
      "/encoder_count", 10,
      [this](std_msgs::msg::Int32::UniquePtr msg){ enc_cb(std::move(msg)); });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 10,
      [this](sensor_msgs::msg::Imu::UniquePtr msg){ imu_cb(std::move(msg)); });

    // 퍼블리셔 생성
    pub_enc_ = create_publisher<std_msgs::msg::Int32>("/encoder_count_fixed", 10);
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_fixed", 10);
  }

private:
  /* ---------- 엔코더 콜백 ---------- */
  void enc_cb(std_msgs::msg::Int32::UniquePtr msg)
  {
    if (first_enc_) {
      enc_offset_ = msg->data;
      first_enc_ = false;
      RCLCPP_INFO(this->get_logger(), "⚙️ 엔코더 오프셋 초기화됨: %d", enc_offset_);
    }

    last_enc_.data = -(msg->data - enc_offset_);
    has_enc_ = true;

    pub_enc_->publish(last_enc_);
    try_print();
  }

  /* ---------- IMU 콜백 ---------- */
  void imu_cb(sensor_msgs::msg::Imu::UniquePtr msg)
  {
    constexpr float G   = 9.80665f;         // 중력가속도: g → m/s²
    constexpr float DEG = M_PI / 180.0f;    // °/s → rad/s

    // 선형 가속도 단위 변환
    msg->linear_acceleration.x *= G;
    msg->linear_acceleration.y *= G;
    msg->linear_acceleration.z *= G;

    // 각속도 단위 변환
    msg->angular_velocity.x *= DEG;
    msg->angular_velocity.y *= DEG;
    msg->angular_velocity.z *= DEG;

    last_imu_ = *msg;
    has_imu_ = true;

    pub_imu_->publish(std::move(msg));
    try_print();
  }

  /* ---------- 동기 출력 ---------- */
  void try_print()
  {
    if (has_enc_ && has_imu_) {
      RCLCPP_INFO(this->get_logger(),
        "Encoder: %d | IMU → Accel: (%.2f, %.2f, %.2f), Gyro: (%.5f, %.5f, %.5f)",
        last_enc_.data,
        last_imu_.linear_acceleration.x,
        last_imu_.linear_acceleration.y,
        last_imu_.linear_acceleration.z,
        last_imu_.angular_velocity.x,
        last_imu_.angular_velocity.y,
        last_imu_.angular_velocity.z
      );

      has_enc_ = false;
      has_imu_ = false;
    }
  }

  /* ---------- 멤버 변수 ---------- */
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  sub_enc_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr     pub_enc_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_imu_;

  int enc_offset_;
  bool first_enc_;
  bool has_enc_, has_imu_;
  std_msgs::msg::Int32 last_enc_;
  sensor_msgs::msg::Imu last_imu_;
};

/* ---------- main 함수 ---------- */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPostProcess>());
  rclcpp::shutdown();
  return 0;
}
