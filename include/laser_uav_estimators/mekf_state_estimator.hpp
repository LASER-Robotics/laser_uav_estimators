#ifndef LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP
#define LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace laser_uav_estimators
{


namespace StateNominal
{
enum
{
  PX         = 0,   // Position x
  PY         = 1,   // Position y
  PZ         = 2,   // Position z
  VX         = 3,   // Velocity x
  VY         = 4,   // Velocity y
  VZ         = 5,   // Velocity z
  QW         = 6,   // Orientation w
  QX         = 7,   // Orientation x
  QY         = 8,   // Orientation y
  QZ         = 9,   // Orientation z
  WX         = 10,  // Angular velocity x
  WY         = 11,  // Angular velocity y
  WZ         = 12,  // Angular velocity z
  TOTAL_SIZE = 13
};
}  // namespace StateNominal

namespace StateError
{
enum
{
  PX         = 0,   // Error Position x
  PY         = 1,   // Error Position y
  PZ         = 2,   // Error Position z
  VX         = 3,   // Error Velocity x
  VY         = 4,   // Error Velocity y
  VZ         = 5,   // Error Velocity z
  ROLL       = 6,   // Error Roll
  PITCH      = 7,   // Error Pitch
  YAW        = 8,   // Error Yaw
  WX         = 9,   // Error Angular velocity x
  WY         = 10,  // Error Angular velocity y
  WZ         = 11,  // Error Angular velocity z
  TOTAL_SIZE = 12
};
}

struct NoiseGains
{
  double position_xy        = 1.0;
  double position_z         = 1.0;
  double orientation        = 1.0;
  double velocity_linear_xy = 1.0;
  double velocity_linear_z  = 1.0;
  double velocity_angular   = 1.0;
};

struct MeasurementNoiseGains
{
  NoiseGains odometry;
};

struct MeasurementPackage
{
  const nav_msgs::msg::Odometry *odometry = nullptr;
  const sensor_msgs::msg::Range *garmin   = nullptr;
};


constexpr float GRAVITY = 9.80665f;

class MEKFEstimator {
public:
  MEKFEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const MeasurementNoiseGains &gains,
                const NoiseGains &default_gains, const std::string &verbosity);
  void predict(const Eigen::VectorXd &u, double dt);

  void correct(const nav_msgs::msg::Odometry measurements);
  void correct(const MeasurementPackage &measurements);

  Eigen::Vector3d         get_position() const;
  Eigen::Quaterniond      get_orientation() const;
  Eigen::Vector3d         get_linear_velocity() const;
  Eigen::Vector3d         get_angular_velocity() const;
  nav_msgs::msg::Odometry get_odometry() const;
  Eigen::MatrixXd         get_covariance() const;
  void                    set_measurement_noise_gains(const MeasurementNoiseGains &gains);
  void                    set_mass(double mass);
  double                  get_mass();


private:
  Eigen::VectorXd x_nominal_;
  Eigen::VectorXd x_nominal_predict;
  Eigen::VectorXd delta_x_;
  Eigen::MatrixXd P_;

  void inject_error_and_reset();

  void set_verbosity(const std::string &verbosity);

  bool is_debug_{false};

  Eigen::Matrix3d    skew_symmetric(const Eigen::Vector3d &v);
  Eigen::Quaterniond exp_SO3_quaternion(const Eigen::Vector3d &theta_vec);

  MeasurementNoiseGains _gains_;
  NoiseGains            _default_gains_;

  float           _mass_;
  Eigen::MatrixXd _allocation_matrix_;
  Eigen::Matrix3d _inertia_;


  rclcpp::Logger logger_;
};

}  // namespace laser_uav_estimators

#endif  // LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP
