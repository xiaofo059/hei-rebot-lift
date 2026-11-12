#include "hei_robot_mapping/map_projection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <vector>
#include <unistd.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>

namespace hei_robot_mapping
{
namespace
{
namespace fs = std::filesystem;
constexpr uint8_t kOccupied = 0, kFree = 254, kUnknown = 205;
// RTAB-Map scans/transforms use float. Include boundary values despite rounding.
constexpr double kHeightEpsilon = 1e-6;
using Point = Eigen::Vector3d;

bool in_height(double z, const ProjectionOptions & o)
{
  return z >= o.z_min - kHeightEpsilon && z <= o.z_max + kHeightEpsilon;
}

bool clip_ray(const Point & start, const Point & end, const ProjectionOptions & o,
  Point & clipped_start, Point & clipped_end)
{
  double low = 0, high = 1;
  const double dz = end.z() - start.z();
  if (std::abs(dz) < 1e-12) {
    if (!in_height(start.z(), o)) {return false;}
  } else {
    const double a = (o.z_min - start.z()) / dz;
    const double b = (o.z_max - start.z()) / dz;
    low = std::max(low, std::min(a, b));
    high = std::min(high, std::max(a, b));
    if (low > high) {return false;}
  }
  clipped_start = start + low * (end - start);
  clipped_end = start + high * (end - start);
  return true;
}

void validate(const ProjectionOptions & o, const std::string & name)
{
  if (!std::isfinite(o.z_min) || !std::isfinite(o.z_max) || o.z_min >= o.z_max ||
    !std::isfinite(o.resolution) || o.resolution <= 0 ||
    !std::isfinite(o.padding) || o.padding < 0 || o.max_cells <= 0)
  {
    throw std::invalid_argument("Require finite z_min < z_max, resolution > 0, padding >= 0, max_cells > 0");
  }
  if (name.empty() || name == "." || name == ".." ||
    std::any_of(name.begin(), name.end(), [](unsigned char c) {
      return c < 32 || c == 127 || c == '/' || c == '\\';
    }))
  {
    throw std::invalid_argument("Output name must be one nonempty filename component");
  }
}

bool path_present(const fs::path & path)
{
  return fs::symlink_status(path).type() != fs::file_type::not_found;
}

struct TemporaryDirectory
{
  fs::path path;
  explicit TemporaryDirectory(const fs::path & parent)
  {
    auto pattern = (parent / ".projection-XXXXXX").string();
    if (!mkdtemp(pattern.data())) {
      throw std::system_error(errno, std::generic_category(), "Create output staging directory");
    }
    path = pattern;
  }
  ~TemporaryDirectory() {std::error_code ignored; fs::remove_all(path, ignored);}
};

template<typename Visitor>
void visit_scans(rtabmap::DBDriver & db, const std::map<int, rtabmap::Transform> & poses,
  Visitor visitor)
{
  for (const auto & item : poses) {
    if (item.first <= 0) {continue;}  // Negative IDs are landmarks, not scan nodes.
    if (item.second.isNull() || !item.second.toEigen4d().allFinite()) {
      throw std::runtime_error("Invalid optimized pose for node " + std::to_string(item.first));
    }
    rtabmap::SensorData data;
    db.getNodeData(item.first, data, false, true, false, false);
    rtabmap::LaserScan scan;
    data.uncompressData(nullptr, nullptr, &scan);
    if (scan.empty()) {continue;}
    if (scan.is2d() || scan.isCompressed() || scan.data().depth() != CV_32F ||
      scan.channels() < 3 || scan.localTransform().isNull())
    {
      throw std::runtime_error("Unsupported/invalid 3-D laser scan at node " + std::to_string(item.first));
    }
    // Saved points are in the scan frame, not necessarily in the body frame.
    const Eigen::Matrix4d transform = (item.second * scan.localTransform()).toEigen4d();
    if (!transform.allFinite()) {throw std::runtime_error("Non-finite scan transform");}
    visitor(scan, transform);
  }
}

template<typename Visitor>
void visit_points(const rtabmap::LaserScan & scan, const Eigen::Matrix4d & transform,
  Visitor visitor)
{
  const Point origin = transform.block<3, 1>(0, 3);
  const auto & matrix = scan.data();
  const int intensity_offset = scan.getIntensityOffset();
  for (int row = 0; row < matrix.rows; ++row) {
    const float * values = matrix.ptr<float>(row);
    for (int col = 0; col < matrix.cols; ++col, values += matrix.channels()) {
      const Point local(values[0], values[1], values[2]);
      if (!local.allFinite()) {continue;}
      const Point point = transform.block<3, 3>(0, 0) * local + origin;
      if (!point.allFinite()) {continue;}
      if (point.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max()) {
        throw std::runtime_error("Map point exceeds PCD float32 coordinate range");
      }
      const float intensity = intensity_offset >= 0 && std::isfinite(values[intensity_offset]) ?
        values[intensity_offset] : 0.0f;
      visitor(origin, point, intensity);
    }
  }
}
}  // namespace

ProjectionResult project_database(const fs::path & database, const fs::path & output_directory,
  const ProjectionOptions & options)
{
  // Use the supplied filename even when it is a symlink to another database.
  const auto output_name = options.output_name.empty() ?
    database.stem().string() : options.output_name;
  validate(options, output_name);
  if (!fs::is_regular_file(database)) {
    throw std::invalid_argument("Database does not exist or is not a file: " + database.string());
  }
  const auto input = fs::canonical(database);
  const auto directory = fs::absolute(output_directory).lexically_normal();
  ProjectionResult result;
  result.pcd_path = directory / (output_name + ".pcd");
  result.pgm_path = directory / (output_name + ".pgm");
  result.yaml_path = directory / (output_name + ".yaml");
  if (path_present(result.pcd_path) || path_present(result.pgm_path) || path_present(result.yaml_path)) {
    throw std::runtime_error("Output already exists (including symlinks); choose another output_name or output_directory");
  }
  std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create(
      {{"DbSqlite3/InMemory", "false"}}));
  if (!db->openConnection(input.string(), false, true)) {
    throw std::runtime_error("Could not open RTAB-Map database read-only");
  }
  const auto poses = db->loadOptimizedPoses();
  if (poses.empty()) {
    throw std::runtime_error("Database has no saved optimized poses; save it with save_map.launch.py first");
  }
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = -min_x;
  const auto bound = [&](const Point & p) {
      min_x = std::min(min_x, p.x()); min_y = std::min(min_y, p.y());
      max_x = std::max(max_x, p.x()); max_y = std::max(max_y, p.y());
    };
  // Two streaming passes keep memory bounded by the grid and a single saved scan.
  visit_scans(*db, poses, [&](const auto & scan, const auto & transform) {
    ++result.scans;
    visit_points(scan, transform, [&](const Point & origin, const Point & point, float) {
      ++result.finite_points;
      if (in_height(point.z(), options)) {++result.height_points; bound(point);}
      Point a, b;
      if (options.raytrace_free_space && clip_ray(origin, point, options, a, b)) {
        bound(a); bound(b);
      }
    });
  });
  if (result.height_points == 0) {
    throw std::runtime_error("No 3-D scan points inside map-frame z_min/z_max; check the height range");
  }
  if (result.finite_points > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("Full map exceeds PCD WIDTH limit (uint32)");
  }
  result.origin_x = std::floor((min_x - options.padding) / options.resolution) * options.resolution;
  result.origin_y = std::floor((min_y - options.padding) / options.resolution) * options.resolution;
  const double width = std::floor((max_x + options.padding - result.origin_x) / options.resolution) + 1;
  const double height = std::floor((max_y + options.padding - result.origin_y) / options.resolution) + 1;
  if (!std::isfinite(width) || !std::isfinite(height) || width < 1 || height < 1 ||
    width > std::numeric_limits<int>::max() || height > std::numeric_limits<int>::max() ||
    static_cast<long double>(width) * height > options.max_cells)
  {
    throw std::runtime_error("Projected grid exceeds max_cells; check map outliers or increase resolution");
  }
  result.width = static_cast<int>(width); result.height = static_cast<int>(height);
  std::vector<uint8_t> pixels(static_cast<size_t>(result.width) * result.height, kUnknown);
  const auto cell = [&](const Point & p) {
      // Bounds were computed from the same transformed scans. Clamp roundoff only.
      return std::pair<int, int>(
        std::clamp(static_cast<int>(std::floor((p.x() - result.origin_x) / options.resolution)), 0, result.width - 1),
        std::clamp(static_cast<int>(std::floor((p.y() - result.origin_y) / options.resolution)), 0, result.height - 1));
    };
  const auto index = [&](int x, int y) {
      return static_cast<size_t>(result.height - 1 - y) * result.width + x;
    };
  fs::create_directories(directory);
  TemporaryDirectory temporary(directory);
  const auto pcd_temp = temporary.path / "map.pcd";
  const auto pgm_temp = temporary.path / "map.pgm", yaml_temp = temporary.path / "map.yaml";
  std::ofstream pcd(pcd_temp, std::ios::binary);
  pcd.exceptions(std::ios::failbit | std::ios::badbit);
  pcd << "# .PCD v0.7 - Point Cloud Data file format\n"
    << "# frame_id: map; optimized poses and scan local transforms applied\n"
    << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    << "WIDTH " << result.finite_points << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
    << result.finite_points << "\nDATA binary\n";
  size_t written_points = 0;
  visit_scans(*db, poses, [&](const auto & scan, const auto & transform) {
    visit_points(scan, transform, [&](const Point & origin, const Point & point, float intensity) {
      // Full 3-D cloud: height clipping applies only to the grid below.
      static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
        "PCD requires IEEE 754 float32");
      const std::array<float, 4> values = {
        static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z()), intensity};
      pcd.write(reinterpret_cast<const char *>(values.data()), sizeof(values));
      ++written_points;
      if (in_height(point.z(), options)) {
        const auto hit = cell(point);
        pixels[index(hit.first, hit.second)] = kOccupied;
      }
      Point a, b;
      if (!options.raytrace_free_space || !clip_ray(origin, point, options, a, b)) {return;}
      const auto start = cell(a), end = cell(b);
      int64_t x = start.first, y = start.second;
      const int64_t dx = std::abs(end.first - start.first), dy = -std::abs(end.second - start.second);
      const int sx = start.first < end.first ? 1 : -1, sy = start.second < end.second ? 1 : -1;
      int64_t error = dx + dy;
      while (true) {
        auto & pixel = pixels[index(static_cast<int>(x), static_cast<int>(y))];
        if (pixel != kOccupied) {pixel = kFree;}  // An obstacle always wins over free rays.
        if (x == end.first && y == end.second) {break;}
        const int64_t twice = 2 * error;
        if (twice >= dy) {error += dy; x += sx;}
        if (twice <= dx) {error += dx; y += sy;}
      }
    });
  });
  pcd.close();
  if (written_points != result.finite_points) {
    throw std::runtime_error("Scan data changed between passes; use a completed database snapshot");
  }
  db->closeConnection(false);
  result.occupied_cells = std::count(pixels.begin(), pixels.end(), kOccupied);
  result.free_cells = std::count(pixels.begin(), pixels.end(), kFree);
  result.unknown_cells = pixels.size() - result.occupied_cells - result.free_cells;
  {
    std::ofstream stream(pgm_temp, std::ios::binary);
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream << "P5\n" << result.width << ' ' << result.height << "\n255\n";
    stream.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    stream.close();
  }
  {
    std::ofstream stream(yaml_temp);
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream << std::setprecision(15)
      << "image: " << std::quoted(result.pgm_path.filename().string()) << '\n'
      << "mode: trinary\nresolution: " << options.resolution
      << "\norigin: [" << result.origin_x << ", " << result.origin_y << ", 0.0]"
      << "\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
      << "# Conversion snapshot; Nav2 uses the standard fields above.\nconversion:\n"
      << "  source_database: " << std::quoted(input.string()) << "\n  frame_id: map\n"
      << "  pcd_file: " << std::quoted(result.pcd_path.filename().string())
      << "\n  pcd_format: binary_xyzi\n  pcd_height_filtered: false\n"
      << "  pcd_points: " << result.finite_points << '\n'
      << "  z_min: " << options.z_min << "\n  z_max: " << options.z_max
      << "\n  resolution: " << options.resolution << "\n  padding: " << options.padding
      << "\n  raytrace_free_space: " << (options.raytrace_free_space ? "true" : "false")
      << "\n  max_cells: " << options.max_cells
      << "\n  scans: " << result.scans << "\n  height_points: " << result.height_points << '\n';
    stream.close();
  }
  // Hard links give no-replace publication even when another converter races us.
  // Publish YAML last, only after both data files exist. Roll back our links on error.
  bool pcd_published = false, pgm_published = false;
  try {
    fs::create_hard_link(pcd_temp, result.pcd_path); pcd_published = true;
    fs::create_hard_link(pgm_temp, result.pgm_path); pgm_published = true;
    fs::create_hard_link(yaml_temp, result.yaml_path);
  } catch (...) {
    std::error_code ignored;
    if (pgm_published) {fs::remove(result.pgm_path, ignored);}
    if (pcd_published) {fs::remove(result.pcd_path, ignored);}
    throw;
  }
  return result;
}
}  // namespace hei_robot_mapping
