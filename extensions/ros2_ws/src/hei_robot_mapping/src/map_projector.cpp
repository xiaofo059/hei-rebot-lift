#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "hei_robot_mapping/map_projection.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_projector");
  int code = 0;
  try {
    auto db_path = node->declare_parameter<std::string>("db_path", "");
    auto name = node->declare_parameter<std::string>("map_name", "");
    const auto directory = node->declare_parameter<std::string>("map_directory",
        "/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map");
    if (!db_path.empty() && !name.empty()) {
      throw std::invalid_argument("Use db_path or legacy map_name, not both");
    }
    if (db_path.empty()) {
      // Keep existing map_name invocations working; new callers can pass any DB path.
      if (name.size() > 3 && name.substr(name.size() - 3) == ".db") {name.resize(name.size() - 3);}
      if (name.empty() || name == "." || name == ".." ||
        name.find_first_of("/\\") != std::string::npos)
      {
        throw std::invalid_argument("Set db_path to a saved .db file (or use legacy map_name)");
      }
      db_path = (std::filesystem::path(directory) / (name + ".db")).string();
    }
    const auto database = std::filesystem::absolute(db_path).lexically_normal();
    auto output_directory = node->declare_parameter<std::string>("output_directory", "");
    if (output_directory.empty()) {output_directory = database.parent_path().string();}
    hei_robot_mapping::ProjectionOptions options;
    options.output_name = node->declare_parameter<std::string>("output_name", "");
    options.z_min = node->declare_parameter<double>("z_min", options.z_min);
    options.z_max = node->declare_parameter<double>("z_max", options.z_max);
    options.resolution = node->declare_parameter<double>("resolution", options.resolution);
    options.padding = node->declare_parameter<double>("padding", options.padding);
    options.raytrace_free_space = node->declare_parameter<bool>(
      "raytrace_free_space", options.raytrace_free_space);
    options.max_cells = node->declare_parameter<int64_t>("max_cells", options.max_cells);
    RCLCPP_INFO(node->get_logger(),
      "Exporting %s read-only: full 3-D PCD, 2-D map Z=[%.3f, %.3f] m, resolution=%.3f m/cell",
      database.c_str(), options.z_min, options.z_max, options.resolution);
    const auto result = hei_robot_mapping::project_database(
      database, output_directory, options);
    RCLCPP_INFO(node->get_logger(),
      "Saved %s, %s and %s; PCD points=%zu, %dx%d, origin=(%.3f, %.3f), scans=%zu, height points=%zu/%zu, "
      "occupied=%zu, free=%zu, unknown=%zu",
      result.pcd_path.c_str(), result.pgm_path.c_str(), result.yaml_path.c_str(),
      result.finite_points, result.width, result.height,
      result.origin_x, result.origin_y, result.scans, result.height_points, result.finite_points,
      result.occupied_cells, result.free_cells, result.unknown_cells);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Map projection failed: %s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
