#include "hei_chassis_controller/protocol.hpp"
#include <algorithm>
#include <cstring>

namespace hei_chassis_controller
{
Frame can_frame(uint16_t id, const std::array<uint8_t, 8> & data)
{
  Frame f{{0x55, 0xaa, 0x1e, 0x03, 0x01, 0, 0, 0, 0x0a, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  f[13] = id & 0xff; f[14] = id >> 8;
  std::copy(data.begin(), data.end(), f.begin() + 21);
  return f;
}
uint32_t read_u32(const uint8_t * d)
{
  return uint32_t(d[0]) | uint32_t(d[1]) << 8 | uint32_t(d[2]) << 16 | uint32_t(d[3]) << 24;
}
float read_float(const uint8_t * d)
{
  uint32_t bits = read_u32(d); float value;
  static_assert(sizeof(float) == 4, "IEEE 754 float required");
  std::memcpy(&value, &bits, 4); return value;
}
Frame velocity_frame(uint16_t id, float velocity)
{
  uint32_t bits; std::memcpy(&bits, &velocity, 4);
  return can_frame(0x200 + id, {{uint8_t(bits), uint8_t(bits >> 8), uint8_t(bits >> 16), uint8_t(bits >> 24), 0, 0, 0, 0}});
}
Frame command_frame(uint16_t id, uint8_t command)
{
  return can_frame(id, {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, command}});
}
Frame register_frame(uint16_t id, uint8_t operation, uint8_t reg, uint32_t value)
{
  return can_frame(0x7ff, {{uint8_t(id), uint8_t(id >> 8), operation, reg,
    uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)}});
}
bool register_reply(const Packet & p, uint16_t slave, uint8_t reg)
{
  return (p.data[0] | uint16_t(p.data[1]) << 8) == slave &&
    (p.data[2] == 0x33 || p.data[2] == 0x55) && p.data[3] == reg;
}
Feedback decode_feedback(const Packet & p, double pmax, double vmax, double tmax)
{
  const auto & d = p.data;
  return {2 * pmax * ((uint16_t(d[1]) << 8) | d[2]) / 65535 - pmax,
    2 * vmax * ((uint16_t(d[3]) << 4) | (d[4] >> 4)) / 4095 - vmax,
    2 * tmax * ((uint16_t(d[4] & 15) << 8) | d[5]) / 4095 - tmax,
    uint8_t(d[0] >> 4)};
}
void PacketParser::feed(const uint8_t * data, size_t count,
  const std::function<void(const Packet &)> & consume)
{
  // Callers bound each serial read to 512 bytes, but arbitrary input remains bounded here.
  for (size_t i = 0; i < count; ++i) {
    pending_.push_back(data[i]);
    while (!pending_.empty() && pending_[0] != 0xaa) {pending_.erase(pending_.begin());}
    if (pending_.size() < 16) {continue;}
    if (pending_[15] != 0x55) {pending_.erase(pending_.begin()); continue;}
    if (pending_[1] == 0x11) {
      Packet p{read_u32(pending_.data() + 3), {}};
      std::copy_n(pending_.begin() + 7, 8, p.data.begin()); consume(p);
    }
    pending_.clear();
  }
}
}  // namespace hei_chassis_controller
