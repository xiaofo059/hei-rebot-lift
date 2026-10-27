#include "hei_robot_mapping/map_snapshot.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <unistd.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap_conversions/MsgConversion.h>

namespace hei_robot_mapping
{
namespace fs = std::filesystem;
namespace
{
class TemporaryDatabase
{
public:
  explicit TemporaryDatabase(const fs::path & destination)
  {
    auto pattern = (destination.parent_path() / ".map-save-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const int fd = mkstemp(buffer.data());
    if (fd < 0) {throw std::runtime_error("Cannot create temporary database: " +
        std::string(std::strerror(errno)));}
    close(fd);
    path = buffer.data();
  }
  ~TemporaryDatabase()
  {
    std::error_code ignored;
    fs::remove(path, ignored);
    fs::remove(path.string() + "-journal", ignored);
    fs::remove(path.string() + "-wal", ignored);
    fs::remove(path.string() + "-shm", ignored);
  }
  fs::path path;
};

bool path_exists(const fs::path & path)
{
  return fs::symlink_status(path).type() != fs::file_type::not_found;
}
}  // namespace

fs::path snapshot_path(const fs::path & directory, const std::string & name)
{
  if (name.empty() || name == "." || name == ".." || name == ".db" ||
    name.find_first_of("/\\") != std::string::npos ||
    name.find_first_not_of(" \t\r\n") == std::string::npos)
  {
    throw std::invalid_argument("map_name must be a non-empty filename, e.g. warehouse_01");
  }
  for (const unsigned char c : name) {
    if (c < 32 || c == 127) {throw std::invalid_argument("map_name contains control characters");}
  }
  if (!directory.is_absolute()) {throw std::invalid_argument("output_dir must be an absolute path");}
  const fs::path result = directory / (fs::path(name).extension() == ".db" ? name : name + ".db");
  if (path_exists(result)) {throw std::runtime_error("Map already exists; use another map_name: " + result.string());}
  return result;
}

SnapshotResult save_snapshot(
  const rtabmap_msgs::msg::MapData & data,
  const rtabmap::ParametersMap & parameters,
  const fs::path & destination)
{
  if (path_exists(destination)) {throw std::runtime_error("Map already exists: " + destination.string());}
  if (data.nodes.empty() || data.graph.poses.empty()) {
    throw std::runtime_error("Mapper returned an empty map; no database was saved");
  }
  if (data.graph.poses_id.size() != data.graph.poses.size()) {
    throw std::runtime_error("Map graph pose arrays have different sizes");
  }
  // A visual dictionary is not supplied by GetMap. Never silently create a
  // database with dangling visual word IDs when used outside this laser setup.
  for (const auto & node : data.nodes) {
    if (!node.word_id_keys.empty()) {
      throw std::runtime_error("This saver supports the current laser-only map, not visual-word maps");
    }
  }
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  std::map<int, rtabmap::Signature> signatures;
  rtabmap::Transform map_to_odom;
  rtabmap_conversions::mapDataFromROS(data, poses, links, signatures, map_to_odom);
  if (signatures.size() != data.nodes.size()) {throw std::runtime_error("Duplicate map node IDs");}
  std::set<int> ids;
  std::set<int> scan_ids;
  for (auto & item : signatures) {
    auto & signature = item.second;
    if (item.first <= 0 || signature.getPose().isNull()) {
      throw std::runtime_error("Map contains an invalid node ID or odometry pose");
    }
    ids.insert(item.first);
    signature.removeLinks();
    signature.setSaved(false);
    auto & sensor = signature.sensorData();
    const auto scan = sensor.laserScanRaw();
    if (sensor.laserScanCompressed().empty() && !scan.empty()) {
      sensor.setLaserScan(rtabmap::LaserScan(
          rtabmap::compressData2(scan.data()), scan.maxPoints(), scan.rangeMax(),
          scan.format(), scan.localTransform()));
    }
    if (!sensor.laserScanCompressed().empty()) {scan_ids.insert(item.first);}
  }
  if (scan_ids.empty()) {throw std::runtime_error("Map contains no saved laser scans; no database was saved");}
  for (const auto & pose : poses) {
    if (pose.second.isNull() || (pose.first > 0 && !ids.count(pose.first))) {
      throw std::runtime_error("Map graph references a missing node or invalid pose");
    }
  }
  auto last_pose = poses.rbegin();
  while (last_pose != poses.rend() && last_pose->first <= 0) {++last_pose;}
  if (last_pose == poses.rend()) {throw std::runtime_error("Map graph has no laser node poses");}
  std::set<std::tuple<int, int, int>> directed_links;
  const auto add_link = [&](const rtabmap::Link & link) {
      const auto key = std::make_tuple(link.from(), link.to(), static_cast<int>(link.type()));
      if (link.from() > 0 && directed_links.insert(key).second) {
        signatures.at(link.from()).addLink(link);
      }
    };
  for (const auto & item : links) {
    const auto & link = item.second;
    if ((link.from() > 0 && !ids.count(link.from())) ||
      (link.to() > 0 && !ids.count(link.to())) || link.transform().isNull())
    {
      throw std::runtime_error("Map constraint references a missing node or invalid transform");
    }
    add_link(link);
    if (link.from() != link.to()) {add_link(link.inverse());}
  }
  fs::create_directories(destination.parent_path());
  TemporaryDatabase temporary(destination);
  auto driver_parameters = parameters;
  driver_parameters[rtabmap::Parameters::kDbSqlite3InMemory()] = "false";
  std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create(driver_parameters));
  if (!db->openConnection(temporary.path.string(), true)) {
    throw std::runtime_error("Cannot open temporary RTAB-Map database");
  }
  // Info must precede Node inserts: loadLastNodes() selects nodes whose
  // time_enter is at least the latest Info time_enter.
  db->addInfoAfterRun(0, *ids.rbegin(), 0, 0, 0, parameters);
  for (const auto & item : signatures) {db->asyncSave(new rtabmap::Signature(item.second));}
  db->emptyTrashes(false);
  db->saveOptimizedPoses(poses, last_pose->second);
  db->closeConnection();

  if (!db->openConnection(temporary.path.string(), false, true)) {
    throw std::runtime_error("Cannot reopen saved database for verification");
  }
  std::set<int> read_ids;
  db->getAllNodeIds(read_ids);
  if (read_ids != ids || db->loadOptimizedPoses().size() != poses.size()) {
    throw std::runtime_error("Saved node/pose count verification failed");
  }
  for (const int id : scan_ids) {
    rtabmap::SensorData sensor;
    db->getNodeData(id, sensor, false, true, false, false);
    if (sensor.laserScanCompressed().empty()) {
      throw std::runtime_error("Saved laser scan verification failed for node " + std::to_string(id));
    }
  }
  std::multimap<int, rtabmap::Link> read_links;
  db->getAllLinks(read_links, false, true);
  if (read_links.size() != directed_links.size()) {
    throw std::runtime_error("Saved constraint count verification failed");
  }
  db->closeConnection(false);
  // Same-directory hard link is an atomic, no-replace publication of the
  // completed database. A concurrent save cannot overwrite the same name.
  if (::link(temporary.path.c_str(), destination.c_str()) != 0) {
    throw std::runtime_error("Cannot publish map (existing name or filesystem error): " +
      std::string(std::strerror(errno)));
  }
  return {destination, ids.size(), scan_ids.size(), directed_links.size()};
}
}  // namespace hei_robot_mapping
