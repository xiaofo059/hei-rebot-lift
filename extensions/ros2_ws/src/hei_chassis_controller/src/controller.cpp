#include "hei_chassis_controller/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <set>
#include <stdexcept>

namespace hei_chassis_controller
{
namespace
{
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;
}
int64_t OmniChassisController::steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Callback OmniChassisController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {"front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint", "front_left_wheel_joint"});
    auto_declare<double>("wheel_radius", 0.076);
    auto_declare<double>("half_length", 0.223584); auto_declare<double>("half_width", 0.223584);
    auto_declare<std::vector<double>>("wheel_signs", {1., 1., 1., 1.});
    auto_declare<double>("max_linear_speed", 0.25); auto_declare<double>("max_angular_speed", 0.5);
    auto_declare<double>("max_wheel_speed", 9); auto_declare<double>("max_wheel_acceleration", 8);
    auto_declare<double>("cmd_vel_timeout", 0.3); auto_declare<double>("publish_rate", 30);
    auto_declare<std::string>("base_frame_id", "base_link"); auto_declare<std::string>("odom_frame_id", "wheel_odom");
    auto_declare<bool>("publish_odom_tf", false);
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<std::string>("odom_topic", "/chassis/odom");
    auto_declare<std::vector<double>>("pose_covariance_diagonal", {0.05, 0.05, 1e6, 1e6, 1e6, 0.1});
    auto_declare<std::vector<double>>("twist_covariance_diagonal", {0.02, 0.02, 1e6, 1e6, 1e6, 0.05});
  } catch (const std::exception & e) {RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what()); return Callback::ERROR;}
  return Callback::SUCCESS;
}
controller_interface::InterfaceConfiguration OmniChassisController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {c.names.push_back(j + "/velocity");} return c;
}
controller_interface::InterfaceConfiguration OmniChassisController::state_interface_configuration() const
{
  return command_interface_configuration();
}
Callback OmniChassisController::on_configure(const rclcpp_lifecycle::State &)
{
  accepting_ = false;
  const auto n = get_node();
  try {
    joints_ = n->get_parameter("joints").as_string_array();
    if (joints_.size() != 4 || std::set<std::string>(joints_.begin(), joints_.end()).size() != 4 ||
      std::any_of(joints_.begin(), joints_.end(), [](const auto & j) {return j.empty();})) {
      throw std::runtime_error("Four distinct wheel joints required in RF/RR/LR/LF order");
    }
    kinematics_.radius = n->get_parameter("wheel_radius").as_double();
    kinematics_.half_length = n->get_parameter("half_length").as_double();
    kinematics_.half_width = n->get_parameter("half_width").as_double();
    auto signs = n->get_parameter("wheel_signs").as_double_array();
    if (signs.size() != 4 || std::any_of(signs.begin(), signs.end(), [](double s) {return s != 1 && s != -1;})) {
      throw std::runtime_error("wheel_signs must contain four values, each +1 or -1");
    }
    std::copy(signs.begin(), signs.end(), kinematics_.signs.begin());
    max_linear_ = n->get_parameter("max_linear_speed").as_double();
    max_yaw_ = n->get_parameter("max_angular_speed").as_double();
    max_wheel_ = n->get_parameter("max_wheel_speed").as_double();
    max_accel_ = n->get_parameter("max_wheel_acceleration").as_double();
    timeout_ = n->get_parameter("cmd_vel_timeout").as_double();
    publish_rate_ = n->get_parameter("publish_rate").as_double();
    for (double x : {kinematics_.radius, kinematics_.half_length, kinematics_.half_width,
      max_linear_, max_yaw_, max_wheel_, max_accel_, timeout_, publish_rate_}) {
      if (!std::isfinite(x) || x <= 0) {throw std::runtime_error("Geometry, limits, timeouts and rates must be finite and positive");}
    }
    if (timeout_ > 1 || publish_rate_ > 200) {throw std::runtime_error("Timeout must be <=1s and publish rate <=200Hz");}
    auto base = n->get_parameter("base_frame_id").as_string();
    auto odom = n->get_parameter("odom_frame_id").as_string();
    if (base.empty() || odom.empty() || base == odom || base[0] == '/' || odom[0] == '/') {
      throw std::runtime_error("Distinct, nonempty TF frame names without leading slash required");
    }
    odometry_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      n->create_publisher<nav_msgs::msg::Odometry>(n->get_parameter("odom_topic").as_string(), rclcpp::QoS(5)));
    auto & msg = odometry_->msg_;
    msg.header.frame_id = odom; msg.child_frame_id = base;
    for (auto key : {"pose_covariance_diagonal", "twist_covariance_diagonal"}) {
      auto values = n->get_parameter(key).as_double_array();
      if (values.size() != 6 || std::any_of(values.begin(), values.end(), [](double v) {return !std::isfinite(v) || v <= 0;})) {
        throw std::runtime_error("Covariance diagonal requires six positive finite values");
      }
      auto & covariance = std::string(key) == "pose_covariance_diagonal" ? msg.pose.covariance : msg.twist.covariance;
      for (size_t i = 0; i < 6; ++i) {covariance[7 * i] = values[i];}
    }
    tf_.reset();
    if (n->get_parameter("publish_odom_tf").as_bool()) {
      tf_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        n->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10)));
      tf_->msg_.transforms.resize(1);
      tf_->msg_.transforms[0].header.frame_id = odom;
      tf_->msg_.transforms[0].child_frame_id = base;
    }
    subscription_ = n->create_subscription<geometry_msgs::msg::Twist>(n->get_parameter("cmd_vel_topic").as_string(),
      rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!accepting_) {return;}
        Command c; c.stamp = steady_ns(); c.twist = {{msg->linear.x, msg->linear.y, msg->angular.z}};
        if (std::any_of(c.twist.begin(), c.twist.end(), [](double v) {return !std::isfinite(v);})) {c.twist.fill(0);}
        received_.writeFromNonRT(c);
      });
  } catch (const std::exception & e) {RCLCPP_ERROR(n->get_logger(), "%s", e.what()); return Callback::ERROR;}
  return Callback::SUCCESS;
}
Callback OmniChassisController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 4 || state_interfaces_.size() != 4) {return Callback::ERROR;}
  for (size_t i = 0; i < 4; ++i) {
    bool found_command = false, found_state = false;
    for (size_t j = 0; j < 4; ++j) {
      if (command_interfaces_[j].get_name() == joints_[i] + "/velocity") {command_indices_[i] = j; found_command = true;}
      if (state_interfaces_[j].get_name() == joints_[i] + "/velocity") {state_indices_[i] = j; found_state = true;}
    }
    if (!found_command || !found_state) {return Callback::ERROR;}
  }
  zero(); pose_ = Pose2D{}; last_publish_ns_ = 0;
  received_.writeFromNonRT(Command{}); accepting_ = true;
  return Callback::SUCCESS;
}
void OmniChassisController::zero()
{
  for (auto & c : command_interfaces_) {c.set_value(0);}
  previous_.fill(0);
}
Callback OmniChassisController::on_deactivate(const rclcpp_lifecycle::State &)
{
  accepting_ = false; zero(); return Callback::SUCCESS;
}
Result OmniChassisController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!accepting_) {zero(); return Result::OK;}
  Wheels measured{};
  for (size_t i = 0; i < 4; ++i) {measured[i] = state_interfaces_[state_indices_[i]].get_value();}
  if (!finite(measured)) {zero(); return Result::ERROR;}
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1) {zero(); return Result::OK;}
  const Command command = *received_.readFromRT();
  const double age = (steady_ns() - command.stamp) * 1e-9;
  if (command.stamp == 0 || age < 0 || age > timeout_ || command.twist == Twist{}) {zero();}
  else {
    auto v = command.twist;
    const double speed = std::hypot(v[0], v[1]);
    if (speed > max_linear_) {v[0] *= max_linear_ / speed; v[1] *= max_linear_ / speed;}
    v[2] = std::clamp(v[2], -max_yaw_, max_yaw_);
    auto target = kinematics_.inverse(v);
    limit_wheels(target, previous_, max_wheel_, max_accel_, dt);
    for (size_t i = 0; i < 4; ++i) {command_interfaces_[command_indices_[i]].set_value(target[i]);}
    previous_ = target;
  }
  const auto velocity = kinematics_.forward(measured);
  pose_.integrate(velocity, dt);
  const auto ns = time.nanoseconds();
  if (last_publish_ns_ != 0 && ns >= last_publish_ns_ && (ns - last_publish_ns_) * 1e-9 < 1 / publish_rate_) {return Result::OK;}
  last_publish_ns_ = ns;
  geometry_msgs::msg::Quaternion q; q.z = std::sin(pose_.yaw / 2); q.w = std::cos(pose_.yaw / 2);
  if (odometry_->trylock()) {
    auto & m = odometry_->msg_; m.header.stamp = time;
    m.pose.pose.position.x = pose_.x; m.pose.pose.position.y = pose_.y; m.pose.pose.orientation = q;
    m.twist.twist.linear.x = velocity[0]; m.twist.twist.linear.y = velocity[1]; m.twist.twist.angular.z = velocity[2];
    odometry_->unlockAndPublish();
  }
  if (tf_ && tf_->trylock()) {
    auto & t = tf_->msg_.transforms[0]; t.header.stamp = time;
    t.transform.translation.x = pose_.x; t.transform.translation.y = pose_.y; t.transform.rotation = q;
    tf_->unlockAndPublish();
  }
  return Result::OK;
}
}  // namespace hei_chassis_controller
PLUGINLIB_EXPORT_CLASS(hei_chassis_controller::OmniChassisController, controller_interface::ControllerInterface)
