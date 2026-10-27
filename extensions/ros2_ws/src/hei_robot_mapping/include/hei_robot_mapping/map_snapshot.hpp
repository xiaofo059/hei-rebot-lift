#ifndef HEI_ROBOT_MAPPING__MAP_SNAPSHOT_HPP_
#define HEI_ROBOT_MAPPING__MAP_SNAPSHOT_HPP_

#include <filesystem>
#include <string>
#include <rtabmap/core/Parameters.h>
#include <rtabmap_msgs/msg/map_data.hpp>

namespace hei_robot_mapping
{
struct SnapshotResult
{
  std::filesystem::path path;
  size_t nodes;
  size_t scans;
  size_t directed_links;
};

// A name is one filename component; an optional .db suffix is accepted.
std::filesystem::path snapshot_path(
  const std::filesystem::path & directory, const std::string & name);

// Export a laser-only GetMap snapshot without touching the running mapper.
// The destination is published atomically after a successful read-back check.
// Existing paths (including symlinks) are never overwritten.
SnapshotResult save_snapshot(
  const rtabmap_msgs::msg::MapData & data,
  const rtabmap::ParametersMap & parameters,
  const std::filesystem::path & destination);
}  // namespace hei_robot_mapping
#endif
