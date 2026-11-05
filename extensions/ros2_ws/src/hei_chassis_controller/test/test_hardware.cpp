// Pseudo-terminal motor emulator. Never opens a physical robot device.
#include "hei_chassis_controller/hardware.hpp"
#include "hei_chassis_controller/transport.hpp"
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

using namespace hei_chassis_controller;
class MotorEmulator
{
public:
  MotorEmulator()
  {
    int slave; char name[128];
    if (openpty(&master_, &slave, name, nullptr, nullptr) != 0) {throw std::runtime_error("openpty");}
    port = name; close(slave); fcntl(master_, F_SETFL, O_NONBLOCK);
    worker_ = std::thread([this]() {run();});
  }
  ~MotorEmulator() {running_ = false; worker_.join(); close(master_);}
  std::string port;
  std::atomic<int> drop_motor{0}, fault_motor{0}, enables{0};
  std::atomic<bool> wrong_range{false};
  std::array<std::atomic<double>, 4> commanded{{0, 0, 0, 0}};
private:
  void reply(int motor, const std::array<uint8_t, 8> & data)
  {
    std::array<uint8_t, 16> f{{0xaa, 0x11, 0, uint8_t(17 + motor), 0, 0, 0}};
    std::copy(data.begin(), data.end(), f.begin() + 7); f[15] = 0x55;
    const auto sent = ::write(master_, f.data(), f.size());
    if (sent != static_cast<ssize_t>(f.size())) {return;}
  }
  void status(int i)
  {
    if (drop_motor == i + 1 || drop_motor == -1) {return;}
    const int state = fault_motor == i + 1 ? 0xa : (enabled_[i] ? 1 : 0);
    const auto vel = uint16_t(std::clamp((commanded[i].load() + 30) * 4095 / 60, 0.0, 4095.0));
    reply(i, {{uint8_t((state << 4) | (i + 1)), 0x80, 0, uint8_t(vel >> 4), uint8_t((vel & 15) << 4 | 8), 0, 30, 30}});
  }
  void handle(const uint8_t * f)
  {
    int id = f[13] | int(f[14]) << 8; const auto * d = f + 21;
    if (id == 0x7ff && d[0] >= 1 && d[0] <= 4) {
      const int i = d[0] - 1; const int reg = d[3];
      uint32_t value = 0;
      if (reg == 9 || reg == 10) {
        auto & storage = reg == 9 ? timeouts_[i] : modes_[i];
        if (d[2] == 0x55) {storage = read_u32(d + 4);} value = storage;
      } else {
        float x = reg == 21 ? 12.5f : (reg == 22 ? (wrong_range ? 50.f : 30.f) : 10.f);
        std::memcpy(&value, &x, 4);
      }
      reply(i, {{d[0], 0, d[2], d[3], uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)}});
    } else if (id >= 0x201 && id <= 0x204) {
      int i = id - 0x201; commanded[i] = read_float(d); status(i);
    } else if (id >= 1 && id <= 4 && d[0] == 0xff) {
      int i = id - 1;
      if (d[7] == 0xfc) {enabled_[i] = true; ++enables;}
      if (d[7] == 0xfd) {enabled_[i] = false; commanded[i] = 0;}
      status(i);
    }
  }
  void run()
  {
    std::vector<uint8_t> pending; uint8_t bytes[512];
    while (running_) {
      auto n = ::read(master_, bytes, sizeof(bytes));
      if (n > 0) {pending.insert(pending.end(), bytes, bytes + n);}
      while (pending.size() >= 30) {
        if (pending[0] != 0x55 || pending[1] != 0xaa) {pending.erase(pending.begin()); continue;}
        handle(pending.data()); pending.erase(pending.begin(), pending.begin() + 30);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  int master_;
  std::array<bool, 4> enabled_{};
  std::array<uint32_t, 4> modes_{}, timeouts_{};
  std::atomic<bool> running_{true};
  std::thread worker_;
};

hardware_interface::HardwareInfo make_info(const std::string & port)
{
  hardware_interface::HardwareInfo info; info.name = "test_chassis"; info.type = "system";
  info.hardware_parameters = {{"port", port}, {"baud_rate", "921600"}, {"io_rate", "100"},
    {"motor_timeout_ticks", "6000"}, {"command_timeout", "0.08"}, {"feedback_timeout", "0.06"},
    {"max_motor_speed", "9"}, {"max_motor_acceleration", "8"}, {"pmax", "12.5"}, {"vmax", "30"}, {"tmax", "10"}};
  for (int i = 0; i < 4; ++i) {
    hardware_interface::ComponentInfo j; j.name = "wheel_" + std::to_string(i);
    j.parameters = {{"motor_id", std::to_string(i + 1)}, {"master_id", std::to_string(i + 17)}, {"motor_to_wheel_ratio", "1"}};
    hardware_interface::InterfaceInfo v; v.name = "velocity"; j.command_interfaces.push_back(v);
    for (std::string name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo s; s.name = name; j.state_interfaces.push_back(s);
    }
    info.joints.push_back(j);
  }
  return info;
}
class HardwareTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
};
TEST_F(HardwareTest, ExclusivePortOwnership)
{
  MotorEmulator emulator;
  SerialTransport one, two; one.open(emulator.port, 921600);
  EXPECT_THROW(two.open(emulator.port, 921600), std::runtime_error);
  one.close(); EXPECT_NO_THROW(two.open(emulator.port, 921600));
}
TEST_F(HardwareTest, CommandsExpireAndMissingMotorLatchesFault)
{
  MotorEmulator emulator; DamiaoChassisSystem h;
  ASSERT_EQ(h.on_init(make_info(emulator.port)), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = h.export_command_interfaces();
  ASSERT_EQ(h.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  for (auto & c : commands) {c.set_value(0.5);}
  ASSERT_EQ(h.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  for (auto & value : emulator.commanded) {EXPECT_GT(value.load(), 0); EXPECT_LE(value.load(), 0.5);}
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  for (auto & value : emulator.commanded) {EXPECT_DOUBLE_EQ(value.load(), 0);}
  emulator.drop_motor = 1;
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(h.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
  for (auto & value : emulator.commanded) {EXPECT_DOUBLE_EQ(value.load(), 0);}
  emulator.drop_motor = 0;
  EXPECT_EQ(h.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  h.on_cleanup(rclcpp_lifecycle::State{});
}
TEST_F(HardwareTest, RegisterRepliesDoNotCountAsMotorFeedback)
{
  MotorEmulator emulator; emulator.drop_motor = -1; DamiaoChassisSystem h;
  ASSERT_EQ(h.on_init(make_info(emulator.port)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(h.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  for (auto & value : emulator.commanded) {EXPECT_DOUBLE_EQ(value.load(), 0);}
}
TEST_F(HardwareTest, IncorrectFeedbackScalingRefusesEnable)
{
  MotorEmulator emulator; emulator.wrong_range = true; DamiaoChassisSystem h;
  ASSERT_EQ(h.on_init(make_info(emulator.port)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(h.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(emulator.enables.load(), 0);
}
TEST_F(HardwareTest, MotorFaultStopsAllFourWheels)
{
  MotorEmulator emulator; DamiaoChassisSystem h;
  ASSERT_EQ(h.on_init(make_info(emulator.port)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(h.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  emulator.fault_motor = 3;
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  EXPECT_EQ(h.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
  for (auto & value : emulator.commanded) {EXPECT_DOUBLE_EQ(value.load(), 0);}
}
