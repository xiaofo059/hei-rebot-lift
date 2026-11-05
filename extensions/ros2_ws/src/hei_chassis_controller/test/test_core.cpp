#include "hei_chassis_controller/kinematics.hpp"
#include "hei_chassis_controller/protocol.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace hei_chassis_controller;
TEST(Kinematics, DirectionAndRoundTrip)
{
  Kinematics k;
  const auto forward = k.inverse({{0.1, 0, 0}});
  EXPECT_LT(forward[0], 0); EXPECT_LT(forward[1], 0);
  EXPECT_GT(forward[2], 0); EXPECT_GT(forward[3], 0);
  const auto left = k.inverse({{0, 0.1, 0}});
  EXPECT_LT(left[0], 0); EXPECT_GT(left[1], 0);
  EXPECT_GT(left[2], 0); EXPECT_LT(left[3], 0);
  for (double w : k.inverse({{0, 0, 0.1}})) {EXPECT_LT(w, 0);}
  for (Twist v : {Twist{{0.2, -0.1, 0.3}}, Twist{{0, 0.2, 0}}, Twist{{0, 0, -0.5}}}) {
    auto recovered = k.forward(k.inverse(v));
    for (size_t i = 0; i < 3; ++i) {EXPECT_NEAR(recovered[i], v[i], 1e-12);}
  }
  k.signs = {{-1, 1, -1, 1}};
  const Twist v{{-0.2, 0.15, -0.4}};
  auto recovered = k.forward(k.inverse(v));
  for (size_t i = 0; i < 3; ++i) {EXPECT_NEAR(recovered[i], v[i], 1e-12);}
}
TEST(Kinematics, LimitingPreservesWheelRatiosAndBoundsAcceleration)
{
  Wheels w{{20, -10, 5, -20}}, zero{};
  limit_wheels(w, zero, 9, 8, 0.01);
  EXPECT_NEAR(w[0], 0.08, 1e-12); EXPECT_NEAR(w[1], -0.04, 1e-12);
  for (double x : w) {EXPECT_LE(std::abs(x), 0.08 + 1e-12);}
  w = {{20, -10, 5, -20}}; limit_wheels(w, zero, 9, 1000, 1);
  EXPECT_DOUBLE_EQ(w[0], 9); EXPECT_DOUBLE_EQ(w[1], -4.5);
  EXPECT_FALSE(finite({{0, 0, std::numeric_limits<double>::quiet_NaN(), 0}}));
}
TEST(Kinematics, OdometryIntegratesCurvedAndLateralMotion)
{
  Pose2D p; p.integrate({{1, 0, 1}}, std::acos(-1.0) / 2);
  EXPECT_NEAR(p.x, 1, 1e-12); EXPECT_NEAR(p.y, 1, 1e-12);
  p = {}; p.integrate({{0, 0.2, 0}}, 2);
  EXPECT_NEAR(p.y, 0.4, 1e-12); EXPECT_NEAR(p.x, 0, 1e-12);
}
TEST(Protocol, MatchesExistingU2canDriver)
{
  auto f = velocity_frame(1, 1.0f);
  EXPECT_EQ(f.size(), 30U); EXPECT_EQ(f[0], 0x55); EXPECT_EQ(f[1], 0xaa);
  EXPECT_EQ(f[13], 1); EXPECT_EQ(f[14], 2); EXPECT_EQ(f[18], 8);
  EXPECT_EQ(f[21], 0); EXPECT_EQ(f[22], 0); EXPECT_EQ(f[23], 0x80); EXPECT_EQ(f[24], 0x3f);
  auto mode = register_frame(2, 0x55, 10, 3);
  EXPECT_EQ(mode[13], 0xff); EXPECT_EQ(mode[14], 7); EXPECT_EQ(mode[21], 2);
  EXPECT_EQ(mode[23], 0x55); EXPECT_EQ(mode[24], 10); EXPECT_EQ(mode[25], 3);
  EXPECT_EQ(command_frame(4, 0xfd)[28], 0xfd);
}
TEST(Protocol, ResynchronizesFragmentedNoisyStream)
{
  PacketParser parser; std::vector<Packet> packets;
  auto consumer = [&](const Packet & p) {packets.push_back(p);};
  const std::array<uint8_t, 16> frame{{0xaa, 0x11, 0, 0x11, 0, 0, 0, 0x11, 0x80, 0, 0x80, 0, 0, 30, 31, 0x55}};
  std::vector<uint8_t> garbage(20000, 0x42);
  parser.feed(garbage.data(), garbage.size(), consumer);
  const uint8_t noise[]{0xaa, 1, 2, 3, 4}; parser.feed(noise, 5, consumer);
  parser.feed(frame.data(), 5, consumer); parser.feed(frame.data() + 5, 11, consumer);
  ASSERT_EQ(packets.size(), 1U); EXPECT_EQ(packets[0].id, 17U);
  auto f = decode_feedback(packets[0], 12.5, 30, 10);
  EXPECT_EQ(f.state, 1); EXPECT_NEAR(f.position, 0, 0.001); EXPECT_NEAR(f.velocity, 0, 0.02);
  auto malformed = frame; malformed[15] = 0;
  parser.feed(malformed.data(), 16, consumer); parser.feed(frame.data(), 16, consumer);
  EXPECT_EQ(packets.size(), 2U);
}
