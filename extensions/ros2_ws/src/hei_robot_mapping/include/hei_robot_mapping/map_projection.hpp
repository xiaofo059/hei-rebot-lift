#ifndef HEI_ROBOT_MAPPING__MAP_PROJECTION_HPP_
#define HEI_ROBOT_MAPPING__MAP_PROJECTION_HPP_

#include <cstdint>
#include <filesystem>
#include <string>

namespace hei_robot_mapping
{
struct ProjectionOptions
{
  double z_min = -0.20;
  double z_max = 0.80;
  double resolution = 0.05;
  double padding = 0.50;
  bool raytrace_free_space = true;
  int64_t max_cells = 25000000;
};

struct ProjectionResult
{
  std::filesystem::path pcd_path;
  std::filesystem::path pgm_path;
  std::filesystem::path yaml_path;
  int width = 0;
  int height = 0;
  double origin_x = 0;
  double origin_y = 0;
  size_t scans = 0;
  size_t finite_points = 0;
  size_t height_points = 0;
  size_t occupied_cells = 0;
  size_t free_cells = 0;
  size_t unknown_cells = 0;
};

// Reads saved optimized poses and 3-D laser scans, without writing the database.
// Exports full map-frame XYZI PCD and a height-filtered 2-D PGM/YAML pair.
// All outputs use the database stem. Existing outputs are never replaced.
ProjectionResult project_database(
  const std::filesystem::path & database,
  const std::filesystem::path & output_directory,
  const ProjectionOptions & options);
}  // namespace hei_robot_mapping
#endif
