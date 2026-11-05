#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include <unistd.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include "hei_robot_mapping/map_projection.hpp"

namespace fs = std::filesystem;
using hei_robot_mapping::ProjectionOptions;
using hei_robot_mapping::project_database;
using rtabmap::Transform;

class MapProjection : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/hei-map-projection-XXXXXX";
    const auto path = mkdtemp(pattern);
    ASSERT_NE(path, nullptr);
    directory = path;
  }
  void TearDown() override {fs::remove_all(directory);}

  fs::path fixture(const std::vector<cv::Vec3f> & points,
    const Transform & pose = Transform::getIdentity(),
    const Transform & local = Transform::getIdentity(), bool optimized = true)
  {
    const auto path = directory / "input.db";
    std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create(
        {{"DbSqlite3/InMemory", "false"}}));
    if (!db->openConnection(path.string(), true)) {throw std::runtime_error("fixture database open");}
    auto signature = new rtabmap::Signature(1, 0, 0, 1.0, "node", Transform::getIdentity());
    cv::Mat scan(1, static_cast<int>(points.size()), CV_32FC3);
    for (size_t i = 0; i < points.size(); ++i) {scan.at<cv::Vec3f>(0, i) = points[i];}
    signature->sensorData().setLaserScan(rtabmap::LaserScan(
        rtabmap::compressData2(scan), static_cast<int>(points.size()), 100.0f,
        rtabmap::LaserScan::kXYZ, local));
    db->asyncSave(signature);
    db->emptyTrashes(false);
    if (optimized) {db->saveOptimizedPoses({{1, pose}}, pose);}
    db->closeConnection();
    return path;
  }

  static std::string bytes(const fs::path & path)
  {
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  static std::vector<unsigned char> pixels(const hei_robot_mapping::ProjectionResult & result)
  {
    std::ifstream stream(result.pgm_path, std::ios::binary);
    std::string magic;
    int width, height, maximum;
    stream >> magic >> width >> height >> maximum;
    EXPECT_EQ(magic, "P5"); EXPECT_EQ(maximum, 255);
    EXPECT_EQ(width, result.width); EXPECT_EQ(height, result.height);
    stream.get();
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  static unsigned char at(const hei_robot_mapping::ProjectionResult & r,
    const std::vector<unsigned char> & image, double x, double y, double resolution)
  {
    const int col = static_cast<int>(std::floor((x - r.origin_x) / resolution));
    const int row = r.height - 1 - static_cast<int>(std::floor((y - r.origin_y) / resolution));
    return image.at(static_cast<size_t>(row) * r.width + col);
  }
  fs::path directory;
};

TEST_F(MapProjection, UsesOptimizedPoseAndScanExtrinsicAndPreservesDatabase)
{
  const auto path = fixture({{0.25f, 0.25f, 0.3f}, {0.75f, 0.75f, 1.0f}},
    Transform(10, 20, 0, 0, 0, 0), Transform(1, 0, 0.2f, 0, 0, 0));
  const auto before = bytes(path);
  ProjectionOptions o; o.raytrace_free_space = false;
  const auto r = project_database(path, directory / "output", "warehouse", o);
  EXPECT_GT(r.origin_x, 10.0); EXPECT_GT(r.origin_y, 19.0);
  EXPECT_EQ(r.height_points, 1u); EXPECT_EQ(r.occupied_cells, 1u);
  EXPECT_EQ(r.free_cells, 0u);
  const auto image = pixels(r);
  EXPECT_EQ(at(r, image, 11.25, 20.25, o.resolution), 0);
  EXPECT_EQ(bytes(path), before);
  const auto yaml = bytes(r.yaml_path);
  EXPECT_NE(yaml.find("image: \"warehouse.pgm\""), std::string::npos);
  EXPECT_NE(yaml.find("mode: trinary"), std::string::npos);
  EXPECT_NE(yaml.find("source_database:"), std::string::npos);
}

TEST_F(MapProjection, InclusiveGlobalHeightLimitsAndPgmYDirection)
{
  const auto path = fixture({{0.25f, 0.25f, -0.2f}, {0.75f, 0.75f, 0.8f},
      {1.25f, 1.25f, -0.201f}, {1.75f, 1.75f, 0.801f},
      {std::numeric_limits<float>::quiet_NaN(), 1, 0}});
  ProjectionOptions o; o.raytrace_free_space = false; o.resolution = 0.5; o.padding = 0;
  const auto r = project_database(path, directory, "height", o);
  EXPECT_EQ(r.finite_points, 4u); EXPECT_EQ(r.height_points, 2u);
  EXPECT_EQ(r.width, 2); EXPECT_EQ(r.height, 2);
  EXPECT_EQ(pixels(r), (std::vector<unsigned char>{205, 0, 0, 205}));
}

TEST_F(MapProjection, HeightFilteringHappensAfterFullPoseRotation)
{
  // Pitch +90 degrees turns scan +X into map -Z. Only the first point is in band.
  const auto path = fixture({{0.5f, 0.25f, 0.25f}, {2.0f, 0.25f, 0.25f}},
    Transform(0, 0, 1, 0, float(3.141592653589793 / 2), 0));
  ProjectionOptions o; o.raytrace_free_space = false;
  const auto r = project_database(path, directory, "rotation", o);
  EXPECT_EQ(r.height_points, 1u);
}

TEST_F(MapProjection, RaytracingPreservesUnknownAndObstaclesWin)
{
  const auto path = fixture({{2.25f, 0.25f, 0}, {4.25f, 0.25f, 0}});
  ProjectionOptions o; o.resolution = 0.5; o.padding = 1;
  const auto r = project_database(path, directory, "rays", o);
  const auto image = pixels(r);
  EXPECT_EQ(at(r, image, 1.25, 0.25, o.resolution), 254);
  EXPECT_EQ(at(r, image, 2.25, 0.25, o.resolution), 0);
  EXPECT_EQ(at(r, image, 4.25, 0.25, o.resolution), 0);
  EXPECT_EQ(at(r, image, 1.25, -0.75, o.resolution), 205);
  EXPECT_GT(r.free_cells, 0u); EXPECT_GT(r.unknown_cells, 0u);
}

TEST_F(MapProjection, RaysOnlyClearTheirIntersectionWithHeightSlab)
{
  const auto path = fixture({{4, 0, -2.5f}, {4, 2, -1.5f}}, Transform(0, 0, 2, 0, 0, 0));
  ProjectionOptions o; o.resolution = 0.25; o.padding = 2.5;
  const auto r = project_database(path, directory, "clipped", o);
  const auto image = pixels(r);
  EXPECT_EQ(at(r, image, 0.1, 0.1, o.resolution), 205);
  EXPECT_EQ(at(r, image, 2.6, 0.1, o.resolution), 254);
  EXPECT_EQ(at(r, image, 4.1, 0.1, o.resolution), 205);
  EXPECT_EQ(at(r, image, 4.1, 2.1, o.resolution), 0);
}

TEST_F(MapProjection, RejectsMissingPosesEmptyBandBadOptionsAndOversizedGrid)
{
  const auto path = fixture({{1, 0, 0}, {4, 2, 0}});
  ProjectionOptions o; o.z_min = 10; o.z_max = 11;
  EXPECT_THROW(project_database(path, directory, "empty", o), std::runtime_error);
  o = ProjectionOptions(); o.z_max = o.z_min;
  EXPECT_THROW(project_database(path, directory, "invalid", o), std::invalid_argument);
  o = ProjectionOptions(); o.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(project_database(path, directory, "invalid", o), std::invalid_argument);
  o = ProjectionOptions(); o.max_cells = 1;
  EXPECT_THROW(project_database(path, directory, "huge", o), std::runtime_error);
  o = ProjectionOptions();
  EXPECT_THROW(project_database(path, directory, "../escape", o), std::invalid_argument);
  EXPECT_THROW(project_database(directory / "missing.db", directory, "missing", o), std::invalid_argument);
  EXPECT_FALSE(fs::exists(directory / "empty.pgm"));
  EXPECT_FALSE(fs::exists(directory / "huge.yaml"));
  fixture({{1, 0, 0}}, Transform::getIdentity(), Transform::getIdentity(), false);
  EXPECT_THROW(project_database(path, directory, "no_poses", o), std::runtime_error);
}

TEST_F(MapProjection, ProtectsExistingFilesAndDanglingSymlinks)
{
  const auto path = fixture({{1, 0, 0}});
  ProjectionOptions o;
  const auto r = project_database(path, directory, "protected", o);
  const auto pgm = bytes(r.pgm_path), yaml = bytes(r.yaml_path);
  EXPECT_THROW(project_database(path, directory, "protected", o), std::runtime_error);
  EXPECT_EQ(bytes(r.pgm_path), pgm); EXPECT_EQ(bytes(r.yaml_path), yaml);
  fs::create_symlink(directory / "does_not_exist", directory / "symlink.yaml");
  EXPECT_THROW(project_database(path, directory, "symlink", o), std::runtime_error);
  EXPECT_TRUE(fs::is_symlink(directory / "symlink.yaml"));
  EXPECT_FALSE(fs::exists(directory / "symlink.pgm"));
}
