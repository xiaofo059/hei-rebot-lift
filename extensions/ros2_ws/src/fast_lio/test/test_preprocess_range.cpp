#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "preprocess.h"
#include <stdexcept>
#include <vector>
#include <iostream>
#include <limits>

void check(double max_range, int stride, const std::vector<float> & input,
           const std::vector<float> & expected, const std::vector<float> & expected_times)
{
  Preprocess processor;
  processor.set(false,AVIA,0.5,stride);
  processor.N_SCANS=4;
  processor.max_range=max_range;
  auto message=std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  message->points.resize(input.size());
  message->point_num=input.size();
  for (size_t i=0;i<input.size();++i)
  {
    message->points[i].x=input[i];
    message->points[i].line=0;
    message->points[i].tag=0x10;
    message->points[i].reflectivity=42;
    message->points[i].offset_time=i*1000000;
  }
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  processor.process(message,output);
  if (output->size()!=expected.size()) throw std::runtime_error("Unexpected retained point count");
  for (size_t i=0;i<expected.size();++i)
    if (output->points[i].x!=expected[i] || output->points[i].curvature!=expected_times[i] ||
        output->points[i].intensity!=42)
      throw std::runtime_error("Range filtering changed point coordinates, timestamps or reflectivity");
}

void check_self_filter(bool enabled, int stride, const std::vector<size_t> &expected)
{
  // Index 0 is the Livox handler's skipped first point; 1-9 are inside/on the box.
  const std::vector<std::array<float, 3>> points = {
    {{0, 0, 0}}, {{-0.25f, 0, 0.4f}},
    {{-0.5f, 0.1f, 0.4f}}, {{0, 0.1f, 0.4f}},
    {{-0.25f, -0.25f, 0.4f}}, {{-0.25f, 0.25f, 0.4f}},
    {{-0.25f, 0, 0}}, {{-0.25f, 0, 0.8f}},
    {{-0.5f, -0.25f, 0}}, {{0, 0.25f, 0.8f}},
    {{-0.501f, 0, 0.4f}}, {{0.001f, 0, 0.4f}},
    {{-0.25f, -0.251f, 0.4f}}, {{-0.25f, 0.251f, 0.4f}},
    {{-0.25f, 0, -0.001f}}, {{-0.25f, 0, 0.801f}},
    {{0.2f, 0, 0}}, {{0.05f, 0, 0}}, {{-0.6f, 0, 0.4f}},
    {{41, 0, 0}}, {{40, 0, 0}}
  };
  Preprocess processor;
  processor.set(false, AVIA, 0.10, stride);
  processor.N_SCANS = 4;
  processor.max_range = 40.0;
  processor.set_self_filter(true, {-0.50, -0.25, 0.0}, {0.0, 0.25, 0.80});
  if (!enabled) processor.set_self_filter(false, {}, {});
  auto message = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  message->point_num = points.size();
  message->points.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    auto &point = message->points[i];
    point.x = points[i][0]; point.y = points[i][1]; point.z = points[i][2];
    point.line = 0; point.tag = 0x10;
    point.reflectivity = 42 + i;
    point.offset_time = i * 1000000;
  }
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  processor.process(message, output);
  if (output->size() != expected.size())
    throw std::runtime_error("Self filter retained an unexpected point count");
  for (size_t i = 0; i < expected.size(); ++i)
  {
    const size_t index = expected[i];
    const auto &point = output->points[i];
    if (point.x != points[index][0] || point.y != points[index][1] || point.z != points[index][2] ||
        point.curvature != static_cast<float>(index) || point.intensity != 42 + index)
      throw std::runtime_error("Self filter changed point selection, XYZ, timestamps or reflectivity");
  }
}

void check_invalid_self_filter()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::vector<double>, std::vector<double>>> invalid = {
    {{}, {0, 0.25, 0.8}}, {{-0.5, -0.25, 0}, {}},
    {{-0.5, -0.25}, {0, 0.25, 0.8}},
    {{0, -0.25, 0}, {0, 0.25, 0.8}},
    {{0.5, -0.25, 0}, {0, 0.25, 0.8}},
    {{nan, -0.25, 0}, {0, 0.25, 0.8}},
    {{-0.5, -0.25, 0}, {0, inf, 0.8}},
    {{-1.e300, -0.25, 0}, {0, 0.25, 0.8}},
    {{1.0, -0.25, 0}, {1.0 + 1.e-10, 0.25, 0.8}}
  };
  for (const auto &bounds : invalid)
  {
    Preprocess processor;
    bool rejected = false;
    try { processor.set_self_filter(true, bounds.first, bounds.second); }
    catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) throw std::runtime_error("Invalid self filter bounds accepted");
  }
}

int main()
{
  try
  {
    const std::vector<float> points={0,0.25,0.5,1,40,41,-40,-41};
    check(40,1,points,{1,40,-40},{3,4,6});
    check(0,1,points,{1,40,41,-40,-41},{3,4,5,6,7});
    check(40,6,{0,1,2,3,4,5,6,7,8,9,10,11,12,13},{6,12},{6,12});
    check_self_filter(true, 1, {10,11,12,13,14,15,16,18,20});
    check_self_filter(true, 6, {12,18});
    check_self_filter(false, 1, {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,20});
    check_invalid_self_filter();
    std::cout << "PASS: range limits, inclusive self filter faces/corners, outside points, disabled filter, "
              << "stride, point metadata and invalid bounds" << std::endl;
  }
  catch (const std::exception & error) { std::cerr << error.what() << std::endl; return 1; }
  return 0;
}
