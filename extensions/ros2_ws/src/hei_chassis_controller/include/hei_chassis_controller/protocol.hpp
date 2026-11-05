#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace hei_chassis_controller
{
struct Packet {uint32_t id; std::array<uint8_t, 8> data;};
using Frame = std::array<uint8_t, 30>;
Frame can_frame(uint16_t id, const std::array<uint8_t, 8> & data);
Frame velocity_frame(uint16_t id, float velocity);
Frame command_frame(uint16_t id, uint8_t command);
Frame register_frame(uint16_t id, uint8_t operation, uint8_t reg, uint32_t value = 0);
uint32_t read_u32(const uint8_t * data);
float read_float(const uint8_t * data);
struct Feedback {double position, velocity, effort; uint8_t state;};
Feedback decode_feedback(const Packet & p, double pmax, double vmax, double tmax);
bool register_reply(const Packet & p, uint16_t slave, uint8_t reg);

// Fixed 16-byte receive frames; retain at most one incomplete frame after parsing.
class PacketParser
{
public:
  void feed(const uint8_t * data, size_t count, const std::function<void(const Packet &)> & consume);
private:
  std::vector<uint8_t> pending_;
};
}  // namespace hei_chassis_controller
