#ifndef LASER_UAV_ESTIMATORS_MULTI_MEKF_STATE_ESTIMATOR_HPP
#define LASER_UAV_ESTIMATORS_MULTI_MEKF_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
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
  QW         = 3,   // Orientation w
  QX         = 4,   // Orientation x
  QY         = 5,   // Orientation y
  QZ         = 6,   // Orientation z
  VX         = 7,   // Velocity x
  VY         = 8,   // Velocity y
  VZ         = 9,   // Velocity z
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
  ROLL       = 3,   // Error Roll
  PITCH      = 4,   // Error Pitch
  YAW        = 5,   // Error Yaw
  VX         = 6,   // Error Velocity x
  VY         = 7,   // Error Velocity y
  VZ         = 8,   // Error Velocity z
  WX         = 9,   // Error Angular velocity x
  WY         = 10,  // Error Angular velocity y
  WZ         = 11,  // Error Angular velocity z
  TOTAL_SIZE = 12
};
}

struct MeasurementPackage
{
  std::optional<nav_msgs::msg::Odometry> px4;
  std::optional<nav_msgs::msg::Odometry> fast_lio;
  std::optional<nav_msgs::msg::Odometry> openvins;
  std::optional<sensor_msgs::msg::Range> garmin;
};

struct Position
{
  double x = 1.0;
  double y = 1.0;
  double z = 1.0;
};

struct Orientation
{
  double roll  = 1.0;
  double pitch = 1.0;
  double yaw   = 1.0;
};

struct LinearVelocity
{
  double vx = 1.0;
  double vy = 1.0;
  double vz = 1.0;
};

struct AngularVelocity
{
  double wx = 1.0;
  double wy = 1.0;
  double wz = 1.0;
};

struct NoiseGains
{
  Position        position;
  Orientation     orientation;
  LinearVelocity  linear_velocity;
  AngularVelocity angular_velocity;
};

struct MeasurementNoiseGains
{
  NoiseGains odometry;
};

constexpr float GRAVITY = -9.80665f;

class MEKFEstimator {
public:
  MEKFEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const MeasurementNoiseGains &noise_px4,
                const MeasurementNoiseGains &noise_fast_lio, const MeasurementNoiseGains &noise_openvins, const MeasurementNoiseGains &noise_garmin,
                const NoiseGains &process_noise, const std::string &verbosity);
  void predict(const Eigen::VectorXd &u, double dt);

  void correct(const MeasurementPackage &measurements);

  Eigen::Vector3d         get_position() const;
  Eigen::Quaterniond      get_orientation() const;
  Eigen::Vector3d         get_linear_velocity() const;
  Eigen::Vector3d         get_angular_velocity() const;
  nav_msgs::msg::Odometry get_odometry() const;
  Eigen::MatrixXd         get_covariance() const;

private:
  // Estruturas de Estado
  Eigen::VectorXd x_nominal_;  // Estado Nominal
  Eigen::VectorXd x_nominal_predict;
  Eigen::VectorXd delta_x_;  // Estado de Erro
  Eigen::MatrixXd P_;        // Covariância do Erro

  void inject_error_and_reset();

  void set_verbosity(const std::string &verbosity);

  bool is_debug_{false};

  Eigen::Matrix3d    skew_symmetric(const Eigen::Vector3d &v);
  Eigen::Quaterniond ExpSO3Quaternion(const Eigen::Vector3d &theta_vec);

  MeasurementNoiseGains _measurement_noise_px4_;
  MeasurementNoiseGains _measurement_noise_fast_lio_;
  MeasurementNoiseGains _measurement_noise_openvins_;
  MeasurementNoiseGains _measurement_noise_garmin_;
  NoiseGains            _process_noise_;

  float           _mass_;
  Eigen::MatrixXd _allocation_matrix_;
  Eigen::Matrix3d _inertia_;

  rclcpp::Logger logger_;
};

}  // namespace laser_uav_estimators

#endif  // LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP