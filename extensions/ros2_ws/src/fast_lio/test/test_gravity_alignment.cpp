#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include <limits>
#include <stdexcept>

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

void require(bool condition, const char * message)
{
  if (!condition) { throw std::runtime_error(message); }
}

void initialize(ImuProcess & processor, Filter & filter, const V3D & acc)
{
  MeasureGroup measurements;
  measurements.lidar.reset(new PointCloudXYZI());
  measurements.lidar_beg_time = 0.0;
  measurements.lidar_end_time = 0.1;
  // Use small batches so the initialization sample threshold can be exercised.
  for (int i=0; i<4; ++i)
  {
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->linear_acceleration.x = acc.x();
    imu->linear_acceleration.y = acc.y();
    imu->linear_acceleration.z = acc.z();
    measurements.imu.push_back(imu);
  }
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  processor.Process(measurements, filter, cloud);
}

void check(const state_ikfom & state, const V3D & acc)
{
  const M3D R = state.rot.toRotationMatrix();
  const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
  require(R.allFinite(), "Non-finite initial attitude");
  require((R.transpose()*R-M3D::Identity()).norm()<1e-10, "Rotation not orthogonal");
  require(std::abs(R.determinant()-1.0)<1e-10, "Improper rotation");
  require((gravity.normalized()+V3D::UnitZ()).norm()<1e-10, "Gravity not along world -Z");
  require(std::abs(gravity.norm()-G_m_s2)<0.002, "Unexpected gravity magnitude");
  require((R*acc.normalized()+gravity.normalized()).norm()<1e-7,
          "Stationary specific force does not cancel world gravity");
  require(state.pos.norm()<1e-12 && state.vel.norm()<1e-12, "Initialization moved origin");
}

int main()
{
  try
  {
    int cases=0;
    // Level, compound tilts, near vertical and upside-down initialization.
    const std::vector<V3D> axes = {V3D::UnitZ(), -V3D::UnitZ(), V3D::UnitX(),
        -V3D::UnitY(), V3D(0.3,-0.4,0.866025403784), V3D(1e-8,0,-1)};
    for (const auto & axis : axes)
    {
      for (double scale : {1.0, 9.81})
      {
        ImuProcess processor; processor.set_initialization(true, 100); Filter filter;
        const V3D acc=axis.normalized()*scale;
        initialize(processor, filter, acc); check(filter.get_x(), acc); ++cases;
        initialize(processor, filter, acc); check(filter.get_x(), acc); ++cases;
      }
    }
    for (int i=0; i<100; ++i)
    {
      const Eigen::Quaterniond q = Eigen::AngleAxisd(i*.057,V3D::UnitZ()) *
          Eigen::AngleAxisd(i*.041,V3D::UnitY()) * Eigen::AngleAxisd(i*.029,V3D::UnitX());
      const V3D acc=q.conjugate()*V3D(0,0,G_m_s2);
      ImuProcess processor; processor.set_initialization(true, 100); Filter filter;
      initialize(processor,filter,acc);check(filter.get_x(),acc);++cases;
    }
    for (double invalid : {0.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
    {
      ImuProcess processor; processor.set_initialization(true, 100); Filter filter;
      initialize(processor, filter, V3D(invalid,0,0));
      require(filter.get_x().rot.toRotationMatrix().allFinite(), "Invalid sample corrupted attitude");
      initialize(processor, filter, V3D(0.3,0.4,0.866025403784));
      check(filter.get_x(), V3D(0.3,0.4,0.866025403784));++cases;
    }
    for (const auto & axis : axes)
    {
      for (double scale : {1.0, 9.81})
      {
        ImuProcess processor; Filter filter;
        processor.set_initialization(false, 100);
        const V3D acc=axis.normalized()*scale;
        initialize(processor,filter,acc);
        const auto state=filter.get_x();
        const V3D gravity(state.grav[0],state.grav[1],state.grav[2]);
        require((state.rot.toRotationMatrix()-M3D::Identity()).norm()<1e-10,
                "Disabled gravity alignment rotated the body frame");
        require((gravity.normalized()+acc.normalized()).norm()<1e-7,
                "Disabled gravity alignment lost gravity direction");
        require(state.pos.norm()<1e-12 && state.vel.norm()<1e-12,
                "Disabled gravity alignment moved origin");
        ++cases;
      }
    }
    {
      ImuProcess processor; Filter filter;
      processor.set_initialization(true,100);
      const V3D expected_acc(0.0123,0.0234,0.0345);
      const V3D expected_gyr(0.0456,0.0567,0.0678);
      processor.set_acc_cov(expected_acc);
      processor.set_gyr_cov(expected_gyr);
      for (int i=0;i<24;++i)
      {
        initialize(processor,filter,V3D::UnitZ());
        require((processor.cov_acc-expected_acc).norm()>1e-5,
                "IMU initialization finished before 100 samples");
      }
      initialize(processor,filter,V3D::UnitZ());
      require((processor.cov_acc-expected_acc).norm()<1e-12 &&
              (processor.cov_gyr-expected_gyr).norm()<1e-12,
              "IMU initialization did not finish at 100 samples");
      ++cases;
    }
    std::cout << "PASS: " << cases << " IMU initialization cases" << std::endl;
  }
  catch (const std::exception & e) { std::cerr << e.what() << std::endl; return 1; }
  return 0;
}
