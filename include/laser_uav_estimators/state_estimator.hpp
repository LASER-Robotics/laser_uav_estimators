#ifndef LASER_UAV_ESTIMATORS_STATE_ESTIMATOR_HPP
#define LASER_UAV_ESTIMATORS_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <optional>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>
#include <laser_uav_lib/kalman_filter/kalman_filter.hpp>
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <type_traits>
#include <sstream>
#include <cmath>

#include <boost/numeric/odeint.hpp>
#include <vector>

namespace laser_uav_estimators
{
namespace State
{
enum
{
  PX = 0,
  PY = 1,
  PZ = 2,
  QW = 3,
  QX = 4,
  QY = 5,
  QZ = 6,
  VX = 7,
  VY = 8,
  VZ = 9,
  WX = 10,
  WY = 11,
  WZ = 12
};
}

constexpr int STATES = 13;

struct MeasurementPackage
{
  std::optional<nav_msgs::msg::Odometry>   openvins;
  std::optional<nav_msgs::msg::Odometry>   fast_lio;
  std::optional<nav_msgs::msg::Odometry>   px4_odometry;
  std::optional<sensor_msgs::msg::Imu>     imu;
  std::optional<geometry_msgs::msg::Point> gps;
  std::optional<double>                    dt;
};

struct ProcessNoiseGains
{
  double position         = 0.01;
  double orientation      = 0.01;
  double linear_velocity  = 0.1;
  double angular_velocity = 0.1;
};

struct MeasurementNoiseGains
{
  ProcessNoiseGains px4_odometry;
  ProcessNoiseGains openvins;
  ProcessNoiseGains fast_lio;
  ProcessNoiseGains imu;
  ProcessNoiseGains gps;
};

class StateEstimator : public laser_uav_lib::EKF<STATES, Eigen::Dynamic, Eigen::Dynamic> {
public:
  StateEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const std::string &verbosity = "INFO");

  ~StateEstimator() = default;

  void predict(const Eigen::VectorXd &u, double dt) override;

  void correct(const MeasurementPackage &measurements);

  void correct(const Eigen::Matrix<double, Eigen::Dynamic, 1> &z) override {
    (void)z;
  }

  void reset();

  const Eigen::Matrix<double, STATES, 1> &get_state() const override {
    return x_;
  }
  const Eigen::Matrix<double, STATES, STATES> &get_covariance() const override {
    return P_;
  }

  void set_process_noise(const Eigen::Matrix<double, STATES, STATES> &Q) {
    Q_ = Q;
  }

  void set_verbosity(const std::string &verbosity);

  std::string get_verbosity() const;

  void set_process_noise_gains(const ProcessNoiseGains &gains);

  void set_measurement_noise_gains(const MeasurementNoiseGains &gains);

  Eigen::Vector3d calculate_custom_attitude_error(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_ref);

private:
  void correct_odometry(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains);

  void correct_imu(const sensor_msgs::msg::Imu &imu);

  void correct_gps(const geometry_msgs::msg::Point &gps);

  template <typename T>
  Eigen::Matrix<T, STATES, 1> state_transition_model(const Eigen::Matrix<T, STATES, 1> &x, const Eigen::Matrix<T, Eigen::Dynamic, 1> &u) const;

  void calculate_jacobian_F(const Eigen::Matrix<double, STATES, 1> &x, const Eigen::VectorXd &u);

  std::optional<std::pair<Eigen::Matrix<double, 13, 1>, Eigen::Matrix<double, 13, 13>>> fuse_odometry(const MeasurementPackage &measurements);

  void update_Q_matrix();

  int processOdometryMeasurement(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains, Eigen::MatrixXd &H, Eigen::VectorXd &z,
                                 Eigen::MatrixXd &R, int current_row);
  int processImuMeasurement(const sensor_msgs::msg::Imu &imu, double dt, Eigen::MatrixXd &H, Eigen::VectorXd &z, Eigen::MatrixXd &R, int current_row);

  Eigen::Vector3d IntegrateVelocityRK4(const Eigen::Vector3d &current_velocity, const Eigen::Vector3d &linear_acceleration, double delta_t);

  double dt_;

  Eigen::Matrix<double, STATES, 1> x_old_;

  int                                      n_inputs_;
  double                                   mass_;
  Eigen::Matrix<double, 4, Eigen::Dynamic> allocation_matrix_;
  Eigen::Matrix3d                          inertia_tensor_;
  Eigen::Matrix3d                          inertia_tensor_inv_;
  static constexpr double                  GRAVITY = 9.80665;

  rclcpp::Logger logger_;
  std::string    verbosity_;
  bool           is_debug_;

  ProcessNoiseGains     q_gains_;
  MeasurementNoiseGains r_gains_;
};
}  // namespace laser_uav_estimators

#endif  // LASER_UAV_ESTIMATORS_state_estimator_HPP
