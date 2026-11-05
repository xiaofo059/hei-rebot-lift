#include "hei_chassis_controller/controller.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <thread>

using namespace hei_chassis_controller;
class ControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  void SetUp() override
  {
    const rclcpp::NodeOptions opts = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("cmd_vel_topic", "/hei_chassis_test/cmd_vel"),
      rclcpp::Parameter("cmd_vel_timeout", 0.08)});
    ASSERT_EQ(c.init("test_chassis", "", opts), controller_interface::return_type::OK);
    ASSERT_EQ(c.on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::SUCCESS);
    const auto names = c.command_interface_configuration().names;
    commands.reserve(4); states.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      auto joint = names[i].substr(0, names[i].find('/'));
      commands.emplace_back(joint, "velocity", &out[i]);
      states.emplace_back(joint, "velocity", &in[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> lc;
    std::vector<hardware_interface::LoanedStateInterface> ls;
    // Reversed handle order must not change physical wheel ordering.
    for (int i = 3; i >= 0; --i) {lc.emplace_back(commands[i]); ls.emplace_back(states[i]);}
    c.assign_interfaces(std::move(lc), std::move(ls));
    ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::SUCCESS);
    node = std::make_shared<rclcpp::Node>("chassis_test_publisher");
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("/hei_chassis_test/cmd_vel", 1);
    executor.add_node(node); executor.add_node(c.get_node()->get_node_base_interface());
    for (int i = 0; i < 100 && publisher->get_subscription_count() == 0; ++i) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0U);
  }
  void TearDown() override {c.on_deactivate(rclcpp_lifecycle::State{}); c.release_interfaces();}
  void send(double x, double y, double yaw)
  {
    geometry_msgs::msg::Twist msg; msg.linear.x = x; msg.linear.y = y; msg.angular.z = yaw;
    publisher->publish(msg);
    for (int i = 0; i < 8; ++i) {executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  }
  void update() {EXPECT_EQ(c.update(rclcpp::Time(10, 0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::OK);}
  OmniChassisController c;
  Wheels out{}, in{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
};
TEST_F(ControllerTest, FreshCommandRampsThenTimesOutToImmediateZero)
{
  update(); EXPECT_EQ(out, Wheels{});
  send(0.1, 0, 0); update();
  EXPECT_LT(out[0], 0); EXPECT_LT(out[1], 0); EXPECT_GT(out[2], 0); EXPECT_GT(out[3], 0);
  EXPECT_NEAR(out[0], -0.08, 1e-10);
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); update(); EXPECT_EQ(out, Wheels{});
}
TEST_F(ControllerTest, LateralRotationAndInvalidCommands)
{
  send(0, 0.1, 0); update(); EXPECT_LT(out[0], 0); EXPECT_GT(out[1], 0); EXPECT_GT(out[2], 0); EXPECT_LT(out[3], 0);
  send(0, 0, 0); update(); EXPECT_EQ(out, Wheels{});
  send(0, 0, 0.2); update(); for (double w : out) {EXPECT_LT(w, 0);}
  send(std::numeric_limits<double>::quiet_NaN(), 0, 0); update(); EXPECT_EQ(out, Wheels{});
}
TEST_F(ControllerTest, DeactivationRequiresFreshCommand)
{
  send(0.1, 0, 0); update(); EXPECT_NE(out, Wheels{});
  c.on_deactivate(rclcpp_lifecycle::State{}); EXPECT_EQ(out, Wheels{});
  c.on_activate(rclcpp_lifecycle::State{}); update(); EXPECT_EQ(out, Wheels{});
  in[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(c.update(rclcpp::Time(10, 0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::ERROR);
  EXPECT_EQ(out, Wheels{});
}
