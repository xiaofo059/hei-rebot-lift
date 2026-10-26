#include <gtest/gtest.h>
#include <limits>
#include "hei_robot_mapping/xyzi_conversion.hpp"

using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

Cloud fixture(uint32_t width = 3, uint32_t height = 1, bool big = false)
{
  Cloud c;
  c.header.frame_id = "body";
  c.header.stamp.sec = 123;
  c.header.stamp.nanosec = 456;
  c.width = width;
  c.height = height;
  c.point_step = 48;
  c.row_step = width * c.point_step + 16;
  c.is_bigendian = big;
  const char * names[] = {"x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z"};
  const uint32_t offsets[] = {0, 4, 8, 16, 20, 24, 28};
  for (size_t i = 0; i < 7; ++i) {
    Field f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1; f.datatype = Field::FLOAT32;
    c.fields.push_back(f);
  }
  c.data.resize(uint64_t(c.row_step) * height, 0);
  return c;
}

void put(Cloud & c, uint32_t index, uint32_t offset, float value)
{
  std::array<uint8_t, 4> bytes;
  std::memcpy(bytes.data(), &value, 4);
  if (c.is_bigendian != hei_robot_mapping::host_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  auto position = uint64_t(index / c.width) * c.row_step +
    uint64_t(index % c.width) * c.point_step + offset;
  std::memcpy(c.data.data() + position, bytes.data(), 4);
}

float read(const Cloud & c, uint32_t index, uint32_t offset)
{
  float value;
  std::memcpy(&value, c.data.data() + uint64_t(index) * c.point_step + offset, 4);
  return value;
}

TEST(XyziConversion, PreservesCoordinatesHeaderAndRemovesNormalsAndNan)
{
  auto c = fixture();
  for (uint32_t i = 0; i < 3; ++i) {
    put(c, i, 0, float(i + 1)); put(c, i, 4, 2.5f);
    put(c, i, 8, -3.0f); put(c, i, 16, float(10 + i));
  }
  put(c, 1, 0, std::numeric_limits<float>::quiet_NaN());
  auto out = hei_robot_mapping::to_xyzi(c, "body");
  EXPECT_EQ(out.header, c.header);
  ASSERT_EQ(out.fields.size(), 4u);
  EXPECT_EQ(out.fields[3].name, "intensity");
  EXPECT_EQ(out.width, 2u);
  EXPECT_EQ(out.point_step, 16u);
  EXPECT_EQ(out.row_step, 32u);
  EXPECT_EQ(out.data.size(), 32u);
  EXPECT_FLOAT_EQ(read(out, 0, 0), 1.0f);
  EXPECT_FLOAT_EQ(read(out, 1, 0), 3.0f);
  EXPECT_FLOAT_EQ(read(out, 1, 4), 2.5f);
  EXPECT_FLOAT_EQ(read(out, 1, 8), -3.0f);
  EXPECT_FLOAT_EQ(read(out, 1, 12), 12.0f);
}

TEST(XyziConversion, HandlesBigEndianOrganizedCloudWithRowPadding)
{
  auto c = fixture(2, 2, !hei_robot_mapping::host_big_endian());
  for (uint32_t i = 0; i < 4; ++i) {
    put(c, i, 0, float(i + 1)); put(c, i, 4, -2.5f); put(c, i, 8, 4.0f);
    put(c, i, 16, 42.0f);
  }
  auto out = hei_robot_mapping::to_xyzi(c, "body");
  ASSERT_EQ(out.width, 4u);
  EXPECT_EQ(out.height, 1u);
  for (uint32_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(read(out, i, 0), float(i + 1));
    EXPECT_FLOAT_EQ(read(out, i, 4), -2.5f);
    EXPECT_FLOAT_EQ(read(out, i, 12), 42.0f);
  }
}

TEST(XyziConversion, MissingIntensityDefaultsToZero)
{
  auto c = fixture(1);
  c.fields.erase(c.fields.begin() + 3);
  put(c, 0, 0, 1.0f);
  auto out = hei_robot_mapping::to_xyzi(c, "body");
  EXPECT_FLOAT_EQ(read(out, 0, 12), 0.0f);
}

TEST(XyziConversion, RejectsWorldFrameAndMalformedInput)
{
  auto c = fixture(1);
  c.header.frame_id = "camera_init";
  EXPECT_THROW(hei_robot_mapping::to_xyzi(c, "body"), std::runtime_error);
  c.header.frame_id = "body";
  c.data.resize(1);
  EXPECT_THROW(hei_robot_mapping::to_xyzi(c, "body"), std::runtime_error);
  c = fixture(1);
  c.fields[0].datatype = Field::FLOAT64;
  EXPECT_THROW(hei_robot_mapping::to_xyzi(c, "body"), std::runtime_error);
}
