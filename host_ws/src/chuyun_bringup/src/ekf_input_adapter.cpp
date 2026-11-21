#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
template<typename Covariance>
void set_diagonal(Covariance & covariance, const std::array<double, 6> & diagonal)
{
  std::fill(covariance.begin(), covariance.end(), 0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 6 + index] = diagonal[index];
  }
}

template<typename Covariance>
void set_diagonal_3d(Covariance & covariance, const std::array<double, 3> & diagonal)
{
  std::fill(covariance.begin(), covariance.end(), 0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 3 + index] = diagonal[index];
  }
}
}  // namespace

class EkfInputAdapter : public rclcpp::Node
{
public:
  EkfInputAdapter()
  : Node("ekf_input_adapter")
  {
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(20);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    // The MCU calibrates the gyro bias once at boot. If the chassis is disturbed
    // during those seconds a large offset is baked in for the whole run
    // (measured up to 17 deg/s). Re-estimate the bias whenever the wheels report
    // standstill, so the gyro stays usable as the EKF yaw-rate source.
    standstill_sec_ = declare_parameter("imu_bias_standstill_sec", 2.0);
    bias_alpha_ = declare_parameter("imu_bias_alpha", 0.05);
    // Wheel speed scale, calibrated by driving a measured 3.25 m straight line:
    // raw odometry reported 3.317 m, i.e. +2.05% -> k = 0.980.
    odom_linear_scale_ = declare_parameter("odom_linear_scale", 1.0);
    // The MCU's x/y/omega convention is inverted w.r.t. ROS (measured: +vx drives
    // backwards, +wz turns right). Commands are negated in cmd_vel_shaper.py;
    // the odometry feedback must be negated here to stay in the ROS frame.
    odom_axis_sign_ = declare_parameter("odom_axis_sign", 1.0);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom/ekf_input", output_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/ekf_input", output_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", input_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr source) {
        const rclcpp::Time now = this->now();
        have_odom_ = true;
        last_odom_ = now;
        const double v = std::hypot(
          source->twist.twist.linear.x, source->twist.twist.linear.y);
        if (v > 0.01 || std::abs(source->twist.twist.angular.z) > 0.02) {
          last_motion_ = now;
        }

        auto message = *source;
        message.child_frame_id = "base_footprint";

        // Convert the MCU frame into the ROS frame and apply the calibrated scale.
        const double lin = odom_linear_scale_ * odom_axis_sign_;
        message.twist.twist.linear.x *= lin;
        message.twist.twist.linear.y *= lin;
        message.twist.twist.angular.z *= odom_axis_sign_;
        message.pose.pose.position.x *= lin;
        message.pose.pose.position.y *= lin;

        // Wheel odometry: x/y and yaw pose, forward/lateral velocity.
        set_diagonal(
          message.pose.covariance,
          {0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.04});
        set_diagonal(
          message.twist.covariance,
          {0.0025, 0.0004, 1.0e6, 1.0e6, 1.0e6, 0.04});
        odom_publisher_->publish(message);
      });

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", input_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr source) {
        const rclcpp::Time now = this->now();
        auto message = *source;
        if (message.header.frame_id.empty()) {
          message.header.frame_id = "imu_link";
        }

        const bool still =
          (now - last_motion_).seconds() > standstill_sec_;
        // Fail safe: without fresh wheel odometry we cannot tell whether the
        // robot is moving, so never touch the bias estimate.
        const bool odom_fresh =
          have_odom_ && (now - last_odom_).seconds() < 1.0;

        if (still && odom_fresh) {
          if (!have_bias_) {
            bias_ = {message.angular_velocity.x,
              message.angular_velocity.y, message.angular_velocity.z};
            have_bias_ = true;
          } else {
            bias_[0] += bias_alpha_ * (message.angular_velocity.x - bias_[0]);
            bias_[1] += bias_alpha_ * (message.angular_velocity.y - bias_[1]);
            bias_[2] += bias_alpha_ * (message.angular_velocity.z - bias_[2]);
          }
          if (++bias_updates_ % 200 == 0) {
            RCLCPP_INFO(
              get_logger(), "imu bias: %+.3f %+.3f %+.3f deg/s",
              bias_[0] * 57.29578, bias_[1] * 57.29578, bias_[2] * 57.29578);
          }
        }

        if (have_bias_) {
          message.angular_velocity.x -= bias_[0];
          message.angular_velocity.y -= bias_[1];
          message.angular_velocity.z -= bias_[2];
        }

        // The MCU currently marks all IMU covariances as unknown (-1).
        // Supply conservative values so robot_localization can use gyro Z.
        set_diagonal_3d(message.orientation_covariance, {0.10, 0.10, 0.04});
        set_diagonal_3d(message.angular_velocity_covariance, {0.04, 0.04, 0.0025});
        set_diagonal_3d(message.linear_acceleration_covariance, {0.10, 0.10, 0.10});
        imu_publisher_->publish(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "EKF inputs ready: /odom -> /odom/ekf_input, /imu -> /imu/ekf_input "
      "(gyro bias re-estimated on standstill)");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  double standstill_sec_ = 2.0;
  double bias_alpha_ = 0.05;
  double odom_linear_scale_ = 1.0;
  double odom_axis_sign_ = 1.0;
  std::array<double, 3> bias_ = {0.0, 0.0, 0.0};
  bool have_bias_ = false;
  bool have_odom_ = false;
  int bias_updates_ = 0;
  rclcpp::Time last_motion_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfInputAdapter>());
  rclcpp::shutdown();
  return 0;
}
