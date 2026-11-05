#include "hei_chassis_controller/hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <set>
#include <stdexcept>

namespace hei_chassis_controller
{
namespace
{
auto logger = rclcpp::get_logger("hei_chassis_hardware");
using Callback = hardware_interface::CallbackReturn;
using Result = hardware_interface::return_type;
}
DamiaoChassisSystem::~DamiaoChassisSystem() {stop(true);}

Callback DamiaoChassisSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != Callback::SUCCESS) {return Callback::ERROR;}
  try {
    if (info_.joints.size() != 4) {throw std::runtime_error("Exactly four wheel joints required, RF/RR/LR/LF order");}
    const auto & p = info_.hardware_parameters;
    port_ = p.at("port"); baud_ = std::stoi(p.at("baud_rate"));
    io_hz_ = std::stoi(p.at("io_rate"));
    watchdog_ticks_ = std::stoi(p.at("motor_timeout_ticks"));
    command_timeout_ = std::stod(p.at("command_timeout"));
    feedback_timeout_ = std::stod(p.at("feedback_timeout"));
    max_speed_ = std::stod(p.at("max_motor_speed"));
    max_accel_ = std::stod(p.at("max_motor_acceleration"));
    pmax_ = std::stod(p.at("pmax")); vmax_ = std::stod(p.at("vmax")); tmax_ = std::stod(p.at("tmax"));
    for (double x : {command_timeout_, feedback_timeout_, max_speed_, max_accel_, pmax_, vmax_, tmax_}) {
      if (!std::isfinite(x) || x <= 0) {throw std::runtime_error("Hardware limits/timeouts must be finite and positive");}
    }
    if (io_hz_ < 20 || io_hz_ > 200 || watchdog_ticks_ < 1000 || watchdog_ticks_ > 10000 ||
      max_speed_ > vmax_ || command_timeout_ > 1 || feedback_timeout_ > 1) {
      throw std::runtime_error("Invalid IO rate, speed or watchdog (motor ticks must be 1000..10000)");
    }
    std::set<int> used;
    for (size_t i = 0; i < 4; ++i) {
      const auto & j = info_.joints[i];
      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "velocity") {
        throw std::runtime_error("Each wheel needs exactly one velocity command interface");
      }
      std::set<std::string> states;
      for (const auto & s : j.state_interfaces) {states.insert(s.name);}
      if (states != std::set<std::string>{"position", "velocity", "effort"} || j.state_interfaces.size() != 3) {
        throw std::runtime_error("Each wheel needs position, velocity and effort states");
      }
      int id = std::stoi(j.parameters.at("motor_id"));
      int master = std::stoi(j.parameters.at("master_id"));
      reduction_[i] = std::stod(j.parameters.at("motor_to_wheel_ratio"));
      if (id < 1 || id > 15 || master <= 15 || master >= 0x200 || !used.insert(id).second ||
        !used.insert(master).second || !std::isfinite(reduction_[i]) || reduction_[i] <= 0) {
        throw std::runtime_error("Invalid/duplicate motor ID, master ID or wheel reduction");
      }
      ids_[i] = id; masters_[i] = master;
    }
  } catch (const std::exception & e) {RCLCPP_ERROR(logger, "%s", e.what()); return Callback::ERROR;}
  return Callback::SUCCESS;
}
std::vector<hardware_interface::StateInterface> DamiaoChassisSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < 4; ++i) {
    result.emplace_back(info_.joints[i].name, "position", &position_[i]);
    result.emplace_back(info_.joints[i].name, "velocity", &velocity_[i]);
    result.emplace_back(info_.joints[i].name, "effort", &effort_[i]);
  }
  return result;
}
std::vector<hardware_interface::CommandInterface> DamiaoChassisSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < 4; ++i) {result.emplace_back(info_.joints[i].name, "velocity", &command_[i]);}
  return result;
}
Callback DamiaoChassisSystem::on_configure(const rclcpp_lifecycle::State &)
{
  stop(true); fault_ = false; seen_.fill(false); feedback_time_.fill({});
  parser_ = PacketParser{};
  try {serial_.open(port_, baud_); opened_ = true;}
  catch (const std::exception & e) {RCLCPP_ERROR(logger, "%s", e.what()); return Callback::ERROR;}
  return Callback::SUCCESS;
}

void DamiaoChassisSystem::consume(const Packet & p)
{
  for (size_t i = 0; i < 4; ++i) {
    if (p.id != masters_[i]) {continue;}
    if (pending_motor_ == int(i) && register_reply(p, ids_[i], uint8_t(pending_reg_))) {
      reply_ = read_u32(p.data.data() + 4); ack_ = true; return;
    }
    // Only genuine enabled/disabled/fault motor status frames can refresh freshness.
    // Register replies contain a zero high nibble in byte 0, unlike ID|state<<4 status.
    const auto state = uint8_t(p.data[0] >> 4);
    if ((p.data[0] & 15) != ids_[i] || (state != 0 && state != 1 && (state < 8 || state > 14))) {return;}
    // An 8-byte register reply starts with full little endian slave ID and op=33/55.
    if (p.data[0] == ids_[i] && p.data[1] == 0 && (p.data[2] == 0x33 || p.data[2] == 0x55)) {return;}
    const auto f = decode_feedback(p, pmax_, vmax_, tmax_);
    if (seen_[i]) {
      received_position_[i] += std::remainder(f.position - raw_position_[i], 2 * pmax_) / reduction_[i];
    } else {received_position_[i] = 0;}
    raw_position_[i] = f.position; seen_[i] = true;
    received_velocity_[i] = f.velocity / reduction_[i];
    received_effort_[i] = f.effort * reduction_[i];
    states_[i] = f.state; feedback_time_[i] = Clock::now(); return;
  }
}
void DamiaoChassisSystem::receive()
{
  serial_.receive(parser_, [this](const Packet & p) {consume(p);});
}
uint32_t DamiaoChassisSystem::transact(size_t motor, uint8_t operation, uint8_t reg, uint32_t value)
{
  pending_motor_ = int(motor); pending_reg_ = reg; ack_ = false;
  serial_.send(register_frame(ids_[motor], operation, reg, value));
  const auto end = Clock::now() + std::chrono::milliseconds(150);
  while (!ack_ && Clock::now() < end) {receive(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  pending_motor_ = pending_reg_ = -1;
  if (!ack_) {throw std::runtime_error("Motor " + std::to_string(ids_[motor]) + " register " + std::to_string(reg) + " reply timeout");}
  return reply_;
}
bool DamiaoChassisSystem::healthy(Clock::time_point now) const
{
  for (size_t i = 0; i < 4; ++i) {
    if (!seen_[i] || states_[i] != 1 || std::chrono::duration<double>(now - feedback_time_[i]).count() > feedback_timeout_) {return false;}
  }
  return true;
}
Callback DamiaoChassisSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!opened_ || fault_) {return Callback::ERROR;}
  command_.fill(0); requested_.fill(0); applied_.fill(0);
  try {
    // Configure RAM only; do not save to flash, clear faults or change encoder zeros.
    for (auto id : ids_) {serial_.send(command_frame(id, 0xfd));}
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); receive();
    for (size_t i = 0; i < 4; ++i) {
      if (transact(i, 0x55, 10, 3) != 3 || transact(i, 0x33, 10) != 3) {
        throw std::runtime_error("Motor failed to enter VEL mode 3");
      }
      // TIMEOUT uses 50 microsecond ticks: 6000 = 300 ms, not 6000 ms.
      if (transact(i, 0x55, 9, watchdog_ticks_) != uint32_t(watchdog_ticks_) ||
        transact(i, 0x33, 9) != uint32_t(watchdog_ticks_)) {
        throw std::runtime_error("Motor CAN watchdog readback mismatch");
      }
      for (auto entry : {std::pair<uint8_t, double>{21, pmax_}, {22, vmax_}, {23, tmax_}}) {
        const auto bits = transact(i, 0x33, entry.first);
        uint8_t bytes[4]{uint8_t(bits), uint8_t(bits >> 8), uint8_t(bits >> 16), uint8_t(bits >> 24)};
        const double actual = read_float(bytes);
        if (!std::isfinite(actual) || std::abs(actual - entry.second) > entry.second * 0.001) {
          throw std::runtime_error("Motor feedback PMAX/VMAX/TMAX differs from hardware.yaml; refusing incorrect units");
        }
      }
    }
    seen_.fill(false);
    for (auto id : ids_) {serial_.send(velocity_frame(id, 0)); serial_.send(command_frame(id, 0xfc));}
    const auto end = Clock::now() + std::chrono::seconds(1);
    while (!healthy(Clock::now()) && Clock::now() < end) {
      for (auto id : ids_) {serial_.send(velocity_frame(id, 0));}
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); receive();
    }
    if (!healthy(Clock::now())) {throw std::runtime_error("Not all four motors returned fresh enabled feedback");}
    position_ = received_position_; velocity_ = received_velocity_; effort_ = received_effort_;
    snapshot_position_ = position_; snapshot_velocity_ = velocity_; snapshot_effort_ = effort_;
    command_time_ = Clock::now(); fault_ = false; running_ = true; active_ = true;
    thread_ = std::thread(&DamiaoChassisSystem::loop, this);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Chassis activation failed: %s", e.what()); fault_ = true; stop(false); return Callback::ERROR;
  }
  return Callback::SUCCESS;
}
void DamiaoChassisSystem::halt_motors()
{
  if (!opened_) {return;}
  // Attempt every wheel even if another send fails.
  for (auto id : ids_) {try {serial_.send(velocity_frame(id, 0));} catch (...) {}}
  for (auto id : ids_) {try {serial_.send(command_frame(id, 0xfd));} catch (...) {}}
}
void DamiaoChassisSystem::stop(bool close_port)
{
  running_ = false;
  if (thread_.joinable()) {thread_.join();}
  halt_motors(); active_ = false; command_.fill(0); applied_.fill(0);
  if (close_port) {serial_.close(); opened_ = false;}
}
Callback DamiaoChassisSystem::on_deactivate(const rclcpp_lifecycle::State &) {stop(false); return Callback::SUCCESS;}
Callback DamiaoChassisSystem::on_cleanup(const rclcpp_lifecycle::State &) {stop(true); return Callback::SUCCESS;}
Callback DamiaoChassisSystem::on_shutdown(const rclcpp_lifecycle::State &) {stop(true); return Callback::SUCCESS;}
Callback DamiaoChassisSystem::on_error(const rclcpp_lifecycle::State &) {stop(true); return Callback::SUCCESS;}

void DamiaoChassisSystem::loop()
{
  const auto tick = std::chrono::microseconds(1000000 / io_hz_);
  auto previous = Clock::now();
  try {
    while (running_) {
      const auto start = Clock::now();
      receive();
      if (!healthy(start)) {
        for (size_t i = 0; i < 4; ++i) {
          if (states_[i] != 1 || std::chrono::duration<double>(start - feedback_time_[i]).count() > feedback_timeout_) {
            RCLCPP_ERROR(logger, "Motor %u state=0x%x feedback age=%.3fs; chassis fault latched", ids_[i], states_[i], std::chrono::duration<double>(start - feedback_time_[i]).count());
          }
        }
        throw std::runtime_error("Motor fault or stale feedback");
      }
      Wheels target{}; Clock::time_point stamp;
      {std::lock_guard<std::mutex> lock(exchange_);
        target = requested_; stamp = command_time_;
        snapshot_position_ = received_position_; snapshot_velocity_ = received_velocity_; snapshot_effort_ = received_effort_;
      }
      for (size_t i = 0; i < 4; ++i) {target[i] *= reduction_[i];}
      const bool stop_now = !finite(target) || std::chrono::duration<double>(start - stamp).count() > command_timeout_ ||
        std::all_of(target.begin(), target.end(), [](double x) {return x == 0;});
      if (stop_now) {applied_.fill(0);} else {
        limit_wheels(target, applied_, max_speed_, max_accel_, std::clamp(std::chrono::duration<double>(start - previous).count(), 0.0, 0.05));
        applied_ = target;
      }
      for (size_t i = 0; i < 4; ++i) {serial_.send(velocity_frame(ids_[i], float(applied_[i])));}
      previous = start;
      std::this_thread::sleep_until(start + tick);
    }
  } catch (const std::exception & e) {fault_ = true; RCLCPP_ERROR(logger, "%s; stopping all wheels", e.what());}
  running_ = false; halt_motors();
}
Result DamiaoChassisSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fault_) {return Result::ERROR;}
  std::lock_guard<std::mutex> lock(exchange_);
  position_ = snapshot_position_; velocity_ = snapshot_velocity_; effort_ = snapshot_effort_;
  return Result::OK;
}
Result DamiaoChassisSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fault_) {return Result::ERROR;}
  if (!active_) {return Result::OK;}
  std::lock_guard<std::mutex> lock(exchange_);
  requested_ = command_; command_time_ = Clock::now();
  return Result::OK;
}
}  // namespace hei_chassis_controller
PLUGINLIB_EXPORT_CLASS(hei_chassis_controller::DamiaoChassisSystem, hardware_interface::SystemInterface)
