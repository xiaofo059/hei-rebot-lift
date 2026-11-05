#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "preprocess.h"
#include <stdexcept>
#include <vector>
#include <iostream>

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

int main()
{
  try
  {
    const std::vector<float> points={0,0.25,0.5,1,40,41,-40,-41};
    check(40,1,points,{1,40,-40},{3,4,6});
    check(0,1,points,{1,40,41,-40,-41},{3,4,5,6,7});
    check(40,6,{0,1,2,3,4,5,6,7,8,9,10,11,12,13},{6,12},{6,12});
    std::cout << "PASS: range limits, disabled range limit, stride and point metadata" << std::endl;
  }
  catch (const std::exception & error) { std::cerr << error.what() << std::endl; return 1; }
  return 0;
}
