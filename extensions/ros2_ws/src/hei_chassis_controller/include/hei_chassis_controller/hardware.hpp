#pragma once
#include "hei_chassis_controller/kinematics.hpp"
#include "hei_chassis_controller/transport.hpp"
#include "hardware_interface/system_interface.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace hei_chassis_controller
{
class DamiaoChassisSystem : public hardware_interface::SystemInterface
{
public:
  ~DamiaoChassisSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Clock = std::chrono::steady_clock;
  void consume(const Packet &);
  void receive();
  uint32_t transact(size_t motor, uint8_t operation, uint8_t reg, uint32_t value = 0);
  void loop();
  void stop(bool close_port);
  void halt_motors();
  bool healthy(Clock::time_point now) const;
  SerialTransport serial_;
  PacketParser parser_;
  std::array<uint16_t, 4> ids_{}, masters_{};
  Wheels reduction_{}, position_{}, velocity_{}, effort_{}, command_{};
  Wheels received_position_{}, received_velocity_{}, received_effort_{}, raw_position_{};
  Wheels requested_{}, applied_{};
  Wheels snapshot_position_{}, snapshot_velocity_{}, snapshot_effort_{};
  std::array<Clock::time_point, 4> feedback_time_{};
  std::array<uint8_t, 4> states_{};
  std::array<bool, 4> seen_{};
  std::mutex exchange_;
  Clock::time_point command_time_{};
  std::thread thread_;
  std::atomic<bool> running_{false}, fault_{false};
  bool opened_{false}, active_{false};
  std::string port_;
  int baud_{921600}, io_hz_{100}, watchdog_ticks_{6000};
  double command_timeout_{0.2}, feedback_timeout_{0.2}, max_speed_{9}, max_accel_{8};
  double pmax_{12.5}, vmax_{30}, tmax_{10};
  int pending_motor_{-1}, pending_reg_{-1};
  bool ack_{false};
  uint32_t reply_{0};
};
}  // namespace hei_chassis_controller
