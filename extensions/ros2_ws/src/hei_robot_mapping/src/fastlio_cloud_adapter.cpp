#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "hei_robot_mapping/xyzi_conversion.hpp"

class FastlioCloudAdapter : public rclcpp::Node
{
public:
  FastlioCloudAdapter() : Node("fastlio_cloud_adapter")
  {
    expected_frame_ = declare_parameter<std::string>("expected_frame", "body");
    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        try {
          auto output = hei_robot_mapping::to_xyzi(*input, expected_frame_);
          if (output.width == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Cloud contains no finite XYZ points; skipping");
            return;
          }
          publisher_->publish(std::move(output));
        } catch (const std::exception & error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
        }
      });
  }
private:
  std::string expected_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioCloudAdapter>());
  rclcpp::shutdown();
  return 0;
}
