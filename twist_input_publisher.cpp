#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class GoalToVelocity : public rclcpp::Node {
public:
  GoalToVelocity() : Node("goal_to_velocity") {
    declare_parameter<double>("goal_x", 2.5);
    declare_parameter<double>("goal_y", 0.0);
    declare_parameter<double>("slowdown_dist", 0.5);
    declare_parameter<double>("steering_slow_threshold", 20.0); // deg
    declare_parameter<double>("Kp", 5.0);
    declare_parameter<double>("Ki", 0.0);
    declare_parameter<double>("Kd", 0.5);

    goal_x_ = get_parameter("goal_x").as_double();
    goal_y_ = get_parameter("goal_y").as_double();
    slowdown_dist_ = get_parameter("slowdown_dist").as_double();
    steering_slow_thresh_ = get_parameter("steering_slow_threshold").as_double();
    Kp_ = get_parameter("Kp").as_double();
    Ki_ = get_parameter("Ki").as_double();
    Kd_ = get_parameter("Kd").as_double();

    motor_pub_ = create_publisher<std_msgs::msg::Float32>("/motor_cmd", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>("/steering_motor_cmd", 10);

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&GoalToVelocity::odom_callback, this, _1));

    last_time_ = now();
    RCLCPP_INFO(get_logger(), "[INIT] 목표: x=%.2f, y=%.2f | 감속 거리=%.2f | 조향 감속=%.1f deg",
                goal_x_, goal_y_, slowdown_dist_, steering_slow_thresh_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (!initial_set_) {
      init_x_ = x;
      init_y_ = y;
      initial_set_ = true;
      RCLCPP_INFO(get_logger(), "[INIT] 초기 위치: (%.2f, %.2f)", init_x_, init_y_);
    }

    double rel_x = x - init_x_;
    double rel_y = y - init_y_;
    double dx = goal_x_ - rel_x;
    double dy = goal_y_ - rel_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double desired_theta = std::atan2(dy, dx);
    double steering_error = desired_theta - yaw;
    while (steering_error > M_PI) steering_error -= 2.0 * M_PI;
    while (steering_error < -M_PI) steering_error += 2.0 * M_PI;

    float steering_deg = static_cast<float>(steering_error * 180.0 / M_PI);

    // --- PID 제어 ---
    auto now_time = now();
    double dt = (now_time - last_time_).seconds();
    last_time_ = now_time;

    integral_ += steering_error * dt;
    double derivative = (steering_error - prev_error_) / dt;
    double output = Kp_ * steering_error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = steering_error;

    float steer_deg_pid = static_cast<float>(output * 180.0 / M_PI);

    // --- 거리 기반 속도 조절 ---
    float pwm = 0.0f;
    if (distance > 1.5) {
      pwm = 40.0f;
    } else if (distance > 0.8) {
      pwm = 30.0f;
    } else if (distance > 0.3) {
      pwm = 20.0f;
    } else {
      pwm = 0.0f;
      if (!goal_reached_) {
        RCLCPP_INFO(get_logger(), "[INFO] 🎯 목표 위치 도달 (%.2f, %.2f)", rel_x, rel_y);
        goal_reached_ = true;
      }
    }

    std_msgs::msg::Float32 motor_msg;
    motor_msg.data = pwm;
    motor_pub_->publish(motor_msg);

    std_msgs::msg::Float32 steer_msg;
    steer_msg.data = steer_deg_pid;
    steering_pub_->publish(steer_msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[INFO] rel=(%.2f, %.2f) | dist=%.2f | pwm=%.1f | steer=%.2f deg (PID)",
      rel_x, rel_y, distance, pwm, steer_deg_pid);
  }

  rclcpp::Time now() {
    return this->get_clock()->now();
  }

  double goal_x_, goal_y_, slowdown_dist_, steering_slow_thresh_;
  double init_x_ = 0.0, init_y_ = 0.0;
  bool initial_set_ = false;
  bool goal_reached_ = false;

  double Kp_, Ki_, Kd_;
  double integral_ = 0.0, prev_error_ = 0.0;
  rclcpp::Time last_time_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToVelocity>());
  rclcpp::shutdown();
  return 0;
}

