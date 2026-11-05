#pragma once
#include "hei_chassis_controller/kinematics.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <atomic>
#include <chrono>

namespace hei_chassis_controller
{
class OmniChassisController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  struct Command {Twist twist{}; int64_t stamp{0};};
  static int64_t steady_ns();
  void zero();
  std::vector<std::string> joints_;
  std::array<size_t, 4> command_indices_{}, state_indices_{};
  Kinematics kinematics_;
  Pose2D pose_;
  Wheels previous_{};
  double max_linear_{0.25}, max_yaw_{0.5}, max_wheel_{9}, max_accel_{8}, timeout_{0.3};
  double publish_rate_{30};
  int64_t last_publish_ns_{0};
  std::atomic<bool> accepting_{false};
  realtime_tools::RealtimeBuffer<Command> received_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odometry_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_;
};
}  // namespace hei_chassis_controller
