#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class WebTfAggregator : public rclcpp::Node
{
public:
  WebTfAggregator()
  : Node("web_tf_aggregator")
  {
    const double publish_rate = declare_parameter<double>("publish_rate", 12.0);
    max_transform_age_ = declare_parameter<double>("max_transform_age", 2.0);

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    // Web telemetry must never apply reliable-delivery backpressure to the
    // robot's localization graph. A browser display can safely drop a frame.
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/web/tf", output_qos);

    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", input_qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        for (const auto & transform : message->transforms) {
          const auto key = std::make_pair(
            transform.header.frame_id, transform.child_frame_id);
          dynamic_transforms_[key] = transform;
        }
      });

    // Static transforms (base_footprint->base_link, base_link->base_laser with
    // the -92 deg lidar mount) live on /tf_static only. Without them the web
    // console cannot resolve the laser pose and draws the scan rotated by 92 deg.
    static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        for (const auto & transform : message->transforms) {
          const auto key = std::make_pair(
            transform.header.frame_id, transform.child_frame_id);
          static_transforms_[key] = transform;
        }
        RCLCPP_INFO_ONCE(
          get_logger(), "received /tf_static (%zu transforms), forwarding to /web/tf",
          static_transforms_.size());
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_snapshot(); });
  }

private:
  void publish_snapshot()
  {
    if (publisher_->get_subscription_count() == 0U ||
      (dynamic_transforms_.empty() && static_transforms_.empty()))
    {
      return;
    }

    const auto current_time = now();
    tf2_msgs::msg::TFMessage snapshot;
    snapshot.transforms.reserve(dynamic_transforms_.size() + static_transforms_.size());

    for (const auto & entry : dynamic_transforms_) {
      const auto & transform = entry.second;
      const rclcpp::Time stamp(transform.header.stamp);
      const double age = (current_time - stamp).seconds();
      if (age <= max_transform_age_) {
        snapshot.transforms.push_back(transform);
      }
    }

    // Static transforms never age out; re-stamp them so downstream consumers
    // that filter by age keep them.
    for (const auto & entry : static_transforms_) {
      auto transform = entry.second;
      transform.header.stamp = current_time;
      snapshot.transforms.push_back(transform);
    }

    if (!snapshot.transforms.empty()) {
      publisher_->publish(snapshot);
    }
  }

  double max_transform_age_{2.0};
  std::map<
    std::pair<std::string, std::string>,
    geometry_msgs::msg::TransformStamped> dynamic_transforms_;
  std::map<
    std::pair<std::string, std::string>,
    geometry_msgs::msg::TransformStamped> static_transforms_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTfAggregator>());
  rclcpp::shutdown();
  return 0;
}
