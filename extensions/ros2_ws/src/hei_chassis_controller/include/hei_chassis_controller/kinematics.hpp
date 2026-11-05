#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hei_chassis_controller
{
using Wheels = std::array<double, 4>;
using Twist = std::array<double, 3>;

// RF, RR, LR, LF. O-type omni wheels have traction directions at 45 degrees.
// This is not the X-type mecanum-wheel formula. Signs match _ChassisRuntime.
struct Kinematics
{
  double radius{0.076};
  double half_length{0.223584};
  double half_width{0.223584};
  Wheels signs{1, 1, 1, 1};

  Wheels inverse(const Twist & v) const
  {
    const double d = std::sqrt(2.0) * radius;
    const double yaw = (half_length + half_width) * v[2];
    return {{(-v[0] - v[1] - yaw) / d * signs[0],
      (-v[0] + v[1] - yaw) / d * signs[1],
      (v[0] + v[1] - yaw) / d * signs[2],
      (v[0] - v[1] - yaw) / d * signs[3]}};
  }

  Twist forward(const Wheels & w) const
  {
    Wheels s{};
    for (size_t i = 0; i < 4; ++i) {s[i] = w[i] / signs[i];}
    const double d = std::sqrt(2.0) * radius / 4;
    return {{d * (-s[0] - s[1] + s[2] + s[3]),
      d * (-s[0] + s[1] + s[2] - s[3]),
      -d * (s[0] + s[1] + s[2] + s[3]) / (half_length + half_width)}};
  }
};

inline bool finite(const Wheels & values)
{
  return std::all_of(values.begin(), values.end(), [](double x) {return std::isfinite(x);});
}

inline void limit_wheels(Wheels & desired, const Wheels & previous,
  double max_speed, double max_accel, double dt)
{
  double peak = 0;
  for (double x : desired) {peak = std::max(peak, std::abs(x));}
  if (peak > max_speed) {for (auto & x : desired) {x *= max_speed / peak;}}
  double max_delta = 0;
  for (size_t i = 0; i < 4; ++i) {max_delta = std::max(max_delta, std::abs(desired[i] - previous[i]));}
  const double scale = max_delta > 0 ? std::min(1.0, max_accel * dt / max_delta) : 1;
  for (size_t i = 0; i < 4; ++i) {desired[i] = previous[i] + scale * (desired[i] - previous[i]);}
}

struct Pose2D
{
  double x{0}, y{0}, yaw{0};
  void integrate(const Twist & v, double dt)
  {
    const double a = v[2] * dt;
    const double s = std::abs(a) < 1e-8 ? 1 - a * a / 6 : std::sin(a) / a;
    const double c = std::abs(a) < 1e-8 ? a / 2 : (1 - std::cos(a)) / a;
    const double dx = dt * (s * v[0] - c * v[1]);
    const double dy = dt * (c * v[0] + s * v[1]);
    x += std::cos(yaw) * dx - std::sin(yaw) * dy;
    y += std::sin(yaw) * dx + std::cos(yaw) * dy;
    yaw = std::remainder(yaw + a, 2 * std::acos(-1.0));
  }
};
}  // namespace hei_chassis_controller
