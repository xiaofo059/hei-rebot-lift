#ifndef HEI_ROBOT_MAPPING__XYZI_CONVERSION_HPP_
#define HEI_ROBOT_MAPPING__XYZI_CONVERSION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace hei_robot_mapping
{
inline bool host_big_endian()
{
  const uint16_t value = 1;
  return reinterpret_cast<const uint8_t *>(&value)[0] == 0;
}

// Preserve acquisition time and body frame, stripping FAST-LIO's unused normals.
inline sensor_msgs::msg::PointCloud2 to_xyzi(
  const sensor_msgs::msg::PointCloud2 & input, const std::string & expected_frame)
{
  if (input.header.frame_id != expected_frame) {
    throw std::runtime_error("Expected cloud frame '" + expected_frame +
      "', received '" + input.header.frame_id + "'; no frame relabeling is performed");
  }
  const uint64_t count = uint64_t(input.width) * input.height;
  if (!count || !input.point_step ||
    count > std::numeric_limits<uint32_t>::max() / 16 ||
    uint64_t(input.row_step) < uint64_t(input.width) * input.point_step ||
    uint64_t(input.row_step) * (input.height - 1) +
      uint64_t(input.width) * input.point_step > input.data.size())
  {
    throw std::runtime_error("Empty or malformed PointCloud2 layout");
  }
  std::array<int64_t, 4> offsets{{-1, -1, -1, -1}};
  const std::array<std::string, 4> names{{"x", "y", "z", "intensity"}};
  for (size_t i = 0; i < names.size(); ++i) {
    for (const auto & field : input.fields) {
      if (field.name == names[i]) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
          uint64_t(field.offset) + sizeof(float) > input.point_step)
        {
          throw std::runtime_error("Invalid FLOAT32 field: " + names[i]);
        }
        offsets[i] = field.offset;
        break;
      }
    }
    if (i < 3 && offsets[i] < 0) {
      throw std::runtime_error("Missing coordinate field: " + names[i]);
    }
  }
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.height = 1;
  output.is_bigendian = host_big_endian();
  output.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(count);
  const bool swap = input.is_bigendian != host_big_endian();
  uint32_t accepted = 0;
  for (uint32_t row = 0; row < input.height; ++row) {
    for (uint32_t col = 0; col < input.width; ++col) {
      const auto * point = input.data.data() + uint64_t(row) * input.row_step +
        uint64_t(col) * input.point_step;
      std::array<float, 4> values{{0, 0, 0, 0}};
      for (size_t i = 0; i < offsets.size(); ++i) {
        if (offsets[i] < 0) {continue;}
        std::array<uint8_t, sizeof(float)> bytes;
        std::memcpy(bytes.data(), point + offsets[i], sizeof(float));
        if (swap) {std::reverse(bytes.begin(), bytes.end());}
        std::memcpy(&values[i], bytes.data(), sizeof(float));
      }
      if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
        !std::isfinite(values[2])) {continue;}
      if (!std::isfinite(values[3])) {values[3] = 0;}
      std::memcpy(output.data.data() + uint64_t(accepted) * 16, values.data(), 16);
      ++accepted;
    }
  }
  output.width = accepted;
  output.row_step = accepted * 16;
  output.data.resize(output.row_step);
  return output;
}
}  // namespace hei_robot_mapping
#endif  // HEI_ROBOT_MAPPING__XYZI_CONVERSION_HPP_
