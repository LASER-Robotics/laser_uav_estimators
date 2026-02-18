#ifndef LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP
#define LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
// #include <logging.hpp>

namespace laser_uav_estimators
{

// Definição de índices para sensores conforme o artigo
namespace SensorIndex
{
enum
{
  SENSOR_INDEX_LIDAR = 0,  // Fast-LIO2 using livox-mid-360
  SENSOR_INDEX_VIO   = 1,  // OpenVINS using D435i
  SENSOR_INDEX_PX4   = 2,  // PX4 Odometry API
  NUM_SENSORS        = 3
};
}

namespace StateNominal
{
enum
{
  PX         = 0,
  PY         = 1,
  PZ         = 2,  // Posição UAV
  VX         = 3,
  VY         = 4,
  VZ         = 5,  // Velocidade UAV
  QW         = 6,
  QX         = 7,
  QY         = 8,
  QZ         = 9,  // Orientação UAV
  BAX        = 10,
  BAY        = 11,
  BAZ        = 12,  // Bias Acel.
  BGX        = 13,
  BGY        = 14,
  BGZ        = 15,  // Bias Giro.
  GX         = 16,
  GY         = 17,
  GZ         = 18,  // Gravidade
  TOTAL_SIZE = 19
};
}  // namespace StateNominal

namespace StateError
{
enum
{
  PX         = 0,
  PY         = 1,
  PZ         = 2,
  VX         = 3,
  VY         = 4,
  VZ         = 5,
  ROLL       = 6,
  PITCH      = 7,
  YAW        = 8,  // delta_theta (angle-axis)
  DBAX       = 9,
  DBAY       = 10,
  DBAZ       = 11,
  DBGX       = 12,
  DBGY       = 13,
  DBGZ       = 14,
  DGX        = 15,
  DGY        = 16,
  DGZ        = 17,
  TOTAL_SIZE = 18
};
}

struct NoiseGains
{
  double position         = 1.0;
  double orientation      = 1.0;
  double velocity_linear  = 1.0;
  double velocity_angular = 1.0;
  double accelerometer    = 1.0;
  double gyroscope        = 1.0;
};

struct ProcessNoiseGains
{
  NoiseGains imu;
  NoiseGains odom[SensorIndex::NUM_SENSORS];
};


struct MeasurementPackage
{
  const nav_msgs::msg::Odometry *openvins = nullptr;
  const nav_msgs::msg::Odometry *fast_lio = nullptr;
  const nav_msgs::msg::Odometry *px4_api  = nullptr;
};


struct InnovationLimits
{
  Eigen::Vector3d pos;  // [m]  limites por eixo (x,y,z)
  Eigen::Vector3d rot;  // [rad] limites por eixo (roll,pitch,yaw)
};

class ESKFEstimator {
public:
  ESKFEstimator(const int num_sensors_with_drift, const NoiseGains &noise_gains, const ProcessNoiseGains &gains, const std::map<int, InnovationLimits> &limits,
                const std::string &verbosity = "INFO");

  // Passo de Predição: Equações (1)-(8) e (11)
  void predict(const sensor_msgs::msg::Imu &imu_measure, double dt);

  // Passo de Correção: Equações (18)-(21) e (31)-(36)
  void correct(const MeasurementPackage &measurements);

  // Getters do Estado Verdadeiro: Equações (22)-(28)
  Eigen::Vector3d         get_position() const;
  Eigen::Quaterniond      get_orientation() const;
  Eigen::Vector3d         get_linear_velocity() const;
  Eigen::Vector3d         get_angular_velocity() const;
  nav_msgs::msg::Odometry get_odometry() const;

private:
  // Estruturas de Estado
  Eigen::VectorXd x_nominal_;  // Estado Nominal
  Eigen::VectorXd delta_x_;    // Estado de Erro
  Eigen::MatrixXd P_;          // Covariância do Erro

  // Injeção de erro e reset: Equações (22)-(28)
  void inject_error_and_reset();

  // Correção por sensor com remoção de drift: Equações (40)-(41)
  void apply_odometry_correction(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains, int sensor_idx);

  // Árbitro para eliminar outliers (Seção III)
  bool               outlier_arbiter(const nav_msgs::msg::Odometry &measure, const int &sensor_type);
  void               set_verbosity(const std::string &verbosity);
  Eigen::Quaterniond ExpSO3Quaternion(const Eigen::Vector3d &theta_vec);

  // Getters de drift por sensor
  Eigen::Vector3d    get_sensor_p_drift(int sensor_idx) const;
  Eigen::Quaterniond get_sensor_q_drift(int sensor_idx) const;

  Eigen::Matrix3d                 skew_symmetric(const Eigen::Vector3d &v);
  sensor_msgs::msg::Imu           last_imu_measure_;
  std::map<int, InnovationLimits> _limits_;


  ProcessNoiseGains _gains_;
  NoiseGains        _default_gains_;


  const int NUM_SENSORS_WITH_DRIFT;

  const int DRIFT_SIZE       = 7;
  const int DRIFT_ERROR_SIZE = 6;

  const int NUM_STATES_NOMINAL;
  const int NUM_STATES_ERROR;

  rclcpp::Logger logger_;
};

}  // namespace laser_uav_estimators

#endif  // LASER_UAV_ESTIMATORS_ESKF_STATE_ESTIMATOR_HPP