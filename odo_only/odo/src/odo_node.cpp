#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class EncoderIMUOdometry : public rclcpp::Node {
public:
  EncoderIMUOdometry()
  : Node("encoder_imu_odom"), imu_init_counter_(0), gyro_bias_z_(0.0)
  {
    declare_parameter<double>("ticks_meter", 8565.0);
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<int>("encoder_min", -32768);
    declare_parameter<int>("encoder_max", 32768);

    ticks_per_meter_ = get_parameter("ticks_meter").as_double();
    odom_frame_id_   = get_parameter("odom_frame_id").as_string();
    base_frame_id_   = get_parameter("base_frame_id").as_string();
    rate_hz_         = get_parameter("rate_hz").as_double();
    encoder_min_     = get_parameter("encoder_min").as_int();
    encoder_max_     = get_parameter("encoder_max").as_int();

    encoder_low_wrap_  = (encoder_max_ - encoder_min_) * 0.3 + encoder_min_;
    encoder_high_wrap_ = (encoder_max_ - encoder_min_) * 0.7 + encoder_min_;

    encoder_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/encoder_count_fixed", 10, std::bind(&EncoderIMUOdometry::encoder_callback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_fixed", 10, std::bind(&EncoderIMUOdometry::imu_callback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&EncoderIMUOdometry::update, this));

    last_time_ = this->now();
  }

private:
  void encoder_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    encoder_ticks_ = msg->data;

    if (!initial_encoder_set_ && encoder_ticks_ != -1) {
      initial_encoder_ = encoder_ticks_;
      initial_encoder_set_ = true;
      RCLCPP_INFO(this->get_logger(), "🎯 초기 엔코더 기준값 설정됨: %d", initial_encoder_);
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (imu_init_counter_ < imu_init_samples_) {
      gyro_bias_z_ += msg->angular_velocity.z;
      imu_init_counter_++;
      if (imu_init_counter_ == imu_init_samples_) {
        gyro_bias_z_ /= imu_init_samples_;
        RCLCPP_INFO(this->get_logger(), "✅ 자이로 바이어스 보정 완료: %.5f rad/s", gyro_bias_z_);
      }
      return;
    }

    angular_velocity_z_ = msg->angular_velocity.z - gyro_bias_z_;
  }

  void update() {
    if (encoder_ticks_ == -1 || imu_init_counter_ < imu_init_samples_ || !initial_encoder_set_) return;

    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    int relative_ticks = encoder_ticks_ - initial_encoder_;

    if (relative_ticks < encoder_low_wrap_ && prev_ticks_ > encoder_high_wrap_) {
      mult_ += 1;
    }
    if (relative_ticks > encoder_high_wrap_ && prev_ticks_ < encoder_low_wrap_) {
      mult_ -= 1;
    }
    prev_ticks_ = relative_ticks;

    int corrected_ticks = relative_ticks + mult_ * (encoder_max_ - encoder_min_);
    int delta_ticks = corrected_ticks - prev_corrected_ticks_;
    prev_corrected_ticks_ = corrected_ticks;

    double distance = delta_ticks / ticks_per_meter_;
    double vx = -distance / dt;
    yaw_ += angular_velocity_z_ * dt;

    x_ += -std::cos(yaw_) * distance;
    y_ += -std::sin(yaw_) * distance;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id = base_frame_id_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = quat;
    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom_pub_->publish(odom);

    debug_counter_++;
    if (debug_counter_ >= static_cast<int>(rate_hz_)) {
      RCLCPP_INFO(this->get_logger(),
        "[ODOM PUB] x: %.2f | y: %.2f | yaw: %.2f deg | vx: %.2f m/s | wz: %.2f rad/s",
        x_, y_, yaw_ * 180.0 / M_PI, vx, angular_velocity_z_);
      debug_counter_ = 0;
    }
  }

  // 내부 변수
  double ticks_per_meter_, rate_hz_;
  std::string odom_frame_id_, base_frame_id_;
  int encoder_ticks_{-1}, prev_ticks_{0};
  int mult_{0}, prev_corrected_ticks_{0};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double angular_velocity_z_{0.0};
  int encoder_min_, encoder_max_;
  double encoder_low_wrap_, encoder_high_wrap_;
  int debug_counter_{0};
  rclcpp::Time last_time_;

  // 초기 offset용
  int initial_encoder_{0};
  bool initial_encoder_set_{false};

  // 자이로 바이어스
  int imu_init_counter_;
  static constexpr int imu_init_samples_ = 30;
  double gyro_bias_z_;

  // ROS 인터페이스
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderIMUOdometry>());
  rclcpp::shutdown();
  return 0;
}

