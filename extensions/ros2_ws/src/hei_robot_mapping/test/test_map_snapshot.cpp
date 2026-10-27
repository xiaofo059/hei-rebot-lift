#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <unistd.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap_conversions/MsgConversion.h>
#include "hei_robot_mapping/map_snapshot.hpp"

namespace fs = std::filesystem;

class MapSnapshot : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/hei-map-snapshot-XXXXXX";
    const auto path = mkdtemp(pattern);
    ASSERT_NE(path, nullptr);
    directory = path;
  }
  void TearDown() override {fs::remove_all(directory);}

  static rtabmap::ParametersMap parameters()
  {
    return {{"Reg/Strategy", "1"}, {"Mem/IncrementalMemory", "true"},
      {"Mem/InitWMWithAllNodes", "true"}, {"Optimizer/Strategy", "1"},
      {"Optimizer/GravitySigma", "0"}, {"Grid/3D", "true"}, {"Grid/CellSize", "0.1"}};
  }

  static rtabmap_msgs::msg::MapData fixture()
  {
    using rtabmap::Transform;
    std::map<int, rtabmap::Signature> signatures;
    std::map<int, Transform> poses;
    for (int id = 1; id <= 2; ++id) {
      const Transform pose(float(id - 1), 0, 0, 0, 0, 0);
      rtabmap::Signature signature(id, 0, 0, 10.0 + id, "node_" + std::to_string(id), pose);
      cv::Mat scan(1, 100, CV_32FC4);
      for (int i = 0; i < scan.cols; ++i) {
        scan.at<cv::Vec4f>(0, i) = cv::Vec4f(float(i % 10), float(i / 10), 1.0f, 42.0f);
      }
      signature.sensorData().setLaserScan(rtabmap::LaserScan(
          rtabmap::compressData2(scan), 100, 30.0f,
          rtabmap::LaserScan::kXYZI, Transform::getIdentity()));
      cv::Mat grid(1, 1, CV_32FC3, cv::Scalar(1, 2, 3));
      signature.sensorData().setOccupancyGrid(
        cv::Mat(), grid, cv::Mat(), 0.1f, cv::Point3f(0, 0, 0));
      signatures.emplace(id, signature);
      poses.emplace(id, Transform(float(id + 4), 0, 0, 0, 0, 0));
    }
    std::multimap<int, rtabmap::Link> links;
    links.emplace(1, rtabmap::Link(1, 2, rtabmap::Link::kNeighbor,
      Transform(1, 0, 0, 0, 0, 0), cv::Mat::eye(6, 6, CV_64FC1) * 100.0));
    rtabmap_msgs::msg::MapData data;
    rtabmap_conversions::mapDataToROS(poses, links, signatures, Transform(5, 0, 0, 0, 0, 0), data);
    return data;
  }
  fs::path directory;
};

TEST_F(MapSnapshot, SavesReloadableDatabaseWithScansGridsGraphAndParameters)
{
  const auto path = hei_robot_mapping::snapshot_path(directory, "warehouse");
  const auto data = fixture();
  const auto result = hei_robot_mapping::save_snapshot(data, parameters(), path);
  EXPECT_EQ(result.nodes, 2u);
  EXPECT_EQ(result.scans, 2u);
  EXPECT_EQ(result.directed_links, 2u);
  ASSERT_GT(fs::file_size(path), 0u);
  std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
  ASSERT_TRUE(db->openConnection(path.string(), false, true));
  std::set<int> ids;
  db->getAllNodeIds(ids);
  EXPECT_EQ(ids, (std::set<int>{1, 2}));
  EXPECT_EQ(db->getLastParameters().at("Reg/Strategy"), "1");
  const auto optimized = db->loadOptimizedPoses();
  ASSERT_EQ(optimized.size(), 2u);
  EXPECT_FLOAT_EQ(optimized.at(1).x(), 5.0f);
  std::unique_ptr<rtabmap::Signature> signature(db->loadSignature(1));
  ASSERT_NE(signature, nullptr);
  EXPECT_FLOAT_EQ(signature->getPose().x(), 0.0f);
  EXPECT_EQ(signature->getLabel(), "node_1");
  EXPECT_EQ(signature->getLinks().size(), 1u);
  rtabmap::SensorData sensor;
  db->getNodeData(1, sensor, false, true, false, true);
  EXPECT_FALSE(sensor.laserScanCompressed().empty());
  EXPECT_FALSE(sensor.gridObstacleCellsCompressed().empty());
  sensor.uncompressData();
  EXPECT_EQ(sensor.laserScanRaw().size(), 100);
  EXPECT_EQ(sensor.laserScanRaw().format(), rtabmap::LaserScan::kXYZI);
  EXPECT_FLOAT_EQ(sensor.laserScanRaw().data().at<cv::Vec4f>(0, 0)[3], 42.0f);
  db->closeConnection(false);

  // Exercise the actual RTAB-Map loader, not only SQLite/file existence.
  auto params = parameters();
  params["Mem/IncrementalMemory"] = "false";
  rtabmap::Rtabmap mapper;
  mapper.init(params, path.string(), false);
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  std::map<int, rtabmap::Signature> nodes;
  mapper.getGraph(poses, links, true, true, &nodes, false, true, false, true, false, false);
  EXPECT_EQ(poses.size(), 2u);
  EXPECT_EQ(nodes.size(), 2u);
  EXPECT_FALSE(nodes.at(1).sensorData().laserScanCompressed().empty());
  mapper.close(false);
  EXPECT_EQ(data.graph.links.size(), 1u);
}

TEST_F(MapSnapshot, NamesCannotEscapeDirectoryAndExistingFilesArePreserved)
{
  EXPECT_EQ(hei_robot_mapping::snapshot_path(directory, "one.db"), directory / "one.db");
  EXPECT_EQ(hei_robot_mapping::snapshot_path(directory, u8"仓库A"), directory / u8"仓库A.db");
  for (const auto & name : {"", " ", ".", "..", ".db", "../one", "a/b", "a\\b", "a\nb"}) {
    EXPECT_THROW(hei_robot_mapping::snapshot_path(directory, name), std::invalid_argument);
  }
  const auto target = directory / "existing.db";
  std::ofstream(target) << "original";
  EXPECT_THROW(hei_robot_mapping::snapshot_path(directory, "existing"), std::runtime_error);
  EXPECT_THROW(hei_robot_mapping::save_snapshot(fixture(), parameters(), target), std::runtime_error);
  std::ifstream input(target);
  std::string text;
  input >> text;
  EXPECT_EQ(text, "original");
  fs::create_symlink(directory / "not_created", directory / "symlink.db");
  EXPECT_THROW(hei_robot_mapping::snapshot_path(directory, "symlink"), std::runtime_error);
}

TEST_F(MapSnapshot, EmptyGraphAndGraphOnlyResponsesDoNotCreateSuccessfulFiles)
{
  auto data = fixture();
  data.nodes.clear();
  EXPECT_THROW(hei_robot_mapping::save_snapshot(data, parameters(), directory / "empty.db"), std::runtime_error);
  data = fixture();
  for (auto & node : data.nodes) {node.data = rtabmap_msgs::msg::SensorData();}
  EXPECT_THROW(hei_robot_mapping::save_snapshot(data, parameters(), directory / "no_scans.db"), std::runtime_error);
  EXPECT_FALSE(fs::exists(directory / "empty.db"));
  EXPECT_FALSE(fs::exists(directory / "no_scans.db"));
}

TEST_F(MapSnapshot, MissingGraphNodesAndVisualMapsAreRejected)
{
  auto data = fixture();
  data.graph.poses_id[0] = 99;
  EXPECT_THROW(hei_robot_mapping::save_snapshot(data, parameters(), directory / "missing.db"), std::runtime_error);
  data = fixture();
  data.nodes[0].word_id_keys.push_back(1);
  EXPECT_THROW(hei_robot_mapping::save_snapshot(data, parameters(), directory / "visual.db"), std::runtime_error);
  EXPECT_TRUE(fs::is_empty(directory));
}
