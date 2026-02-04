#include <laser_uav_estimators/eskf_state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace laser_uav_estimators
{

/* ErrorStateEstimator() //{ */

ErrorStateEstimator::ErrorStateEstimator(const int num_sensors_with_drift, const NoiseGains &noise_gains, const ProcessNoiseGains &gains,
                                         const std::map<int, InnovationLimits> &limits, const std::string &verbosity)
    : NUM_SENSORS_WITH_DRIFT(num_sensors_with_drift),
      NUM_STATES_NOMINAL(StateNominal::TOTAL_SIZE + (num_sensors_with_drift * DRIFT_SIZE)),
      NUM_STATES_ERROR(StateError::TOTAL_SIZE + (num_sensors_with_drift * DRIFT_ERROR_SIZE)),
      logger_(rclcpp::get_logger("eskf_state_estimator")) {

  set_verbosity(verbosity);

  _default_gains_ = noise_gains;
  _gains_         = gains;
  _limits_        = limits;

  // Inicializa o vetor de 18 + (7 * num_sensors) elementos
  // Ordem: [p, v, q, a_b, w_b, g, p_drift_1, q_drift_1, ...]
  x_nominal_ = Eigen::VectorXd::Zero(NUM_STATES_NOMINAL);
  delta_x_   = Eigen::VectorXd::Zero(NUM_STATES_ERROR);

  // 1. Estados de Orientação (Quatérnios Unitários)
  x_nominal_(StateNominal::QW) = 1.0;

  // 2. Gravidade (Inicializada conforme orientação inicial ou valor padrão)
  x_nominal_.segment<3>(StateNominal::GX) << 0.0, 0.0, -9.81;

  // 3. Drifts Iniciais (Cada sensor começa com drift zero e quatérnio unitário)
  for (int i = 0; i < NUM_SENSORS_WITH_DRIFT; ++i) {
    int q_offset         = StateNominal::TOTAL_SIZE + (i * DRIFT_SIZE) + 3;
    x_nominal_(q_offset) = 1.0;  // qw do drift do sensor i
  }

  RCLCPP_INFO(logger_, "Estimator initialized. NUM_SENSORS: %d, STATES_NOMINAL: %d, STATES_ERROR: %d", NUM_SENSORS_WITH_DRIFT, NUM_STATES_NOMINAL,
              NUM_STATES_ERROR);

  // P é uma matriz quadrada (Error State Size: 18 + 6 * num_sensors)
  P_ = Eigen::MatrixXd::Identity(NUM_STATES_ERROR, NUM_STATES_ERROR);

  // Ajuste de incertezas iniciais típicas
  P_.block<3, 3>(0, 0) *= 0.01;   // Erro de Posição (delta_p)
  P_.block<3, 3>(3, 3) *= 0.01;   // Erro de Velocidade (delta_v)
  P_.block<3, 3>(6, 6) *= 0.001;  // Erro de Orientação (delta_theta)

  // Biases (Incerteza baseada no datasheet da IMU)
  P_.block<3, 3>(9, 9) *= 0.0001;    // delta_ba
  P_.block<3, 3>(12, 12) *= 0.0001;  // delta_bw
  P_.block<3, 3>(15, 15) *= 0.0001;  // delta_g

  // Incerteza do Drift dos Sensores
  for (int i = 0; i < NUM_SENSORS_WITH_DRIFT; ++i) {
    int offset = StateError::TOTAL_SIZE + (i * DRIFT_ERROR_SIZE);
    P_.block<3, 3>(offset, offset) *= 0.1;           // delta_p_drift
    P_.block<3, 3>(offset + 3, offset + 3) *= 0.01;  // delta_theta_drift
  }
}

void ErrorStateEstimator::predict(const sensor_msgs::msg::Imu &imu_measure, double dt) {
  RCLCPP_DEBUG_STREAM(logger_, "[predict] dt: " << dt);
  RCLCPP_DEBUG_STREAM(logger_, "[predict] IMU acc: [" << imu_measure.linear_acceleration.x << ", " << imu_measure.linear_acceleration.y << ", "
                                                      << imu_measure.linear_acceleration.z << "] gyro: [" << imu_measure.angular_velocity.x << ", "
                                                      << imu_measure.angular_velocity.y << ", " << imu_measure.angular_velocity.z << "]");
  RCLCPP_DEBUG_STREAM(logger_, "[predict] x_nominal_ (antes):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  Eigen::Vector3d a_m(imu_measure.linear_acceleration.x, imu_measure.linear_acceleration.y, imu_measure.linear_acceleration.z);
  Eigen::Vector3d w_m(imu_measure.angular_velocity.x, imu_measure.angular_velocity.y, imu_measure.angular_velocity.z);

  last_imu_measure_ = imu_measure;

  // 2. Recuperação de estados atuais do x_nominal_
  Eigen::Vector3d    p = x_nominal_.segment<3>(StateNominal::PX);
  Eigen::Vector3d    v = x_nominal_.segment<3>(StateNominal::VX);
  Eigen::Quaterniond q(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  Eigen::Vector3d    a_b = x_nominal_.segment<3>(StateNominal::BAX);
  Eigen::Vector3d    w_b = x_nominal_.segment<3>(StateNominal::BGX);
  Eigen::Vector3d    g   = x_nominal_.segment<3>(StateNominal::GX);

  // 3. Atualização do Estado Nominal (Equações 1, 2 e 3)
  // v = v + (R(q)*(a_m - a_b) + g) * dt
  Eigen::Matrix3d R = q.toRotationMatrix();

  Eigen::Vector3d acc_corrigida = R * (a_m - a_b) + g;
  // p = p + v*dt + 0.5*acc*dt^2
  x_nominal_.segment<3>(StateNominal::PX) = p + v * dt + 0.5 * acc_corrigida * dt * dt;
  x_nominal_.segment<3>(StateNominal::VX) = v + acc_corrigida * dt;
  RCLCPP_DEBUG_STREAM(logger_, "[predict] acc_corrigida: [" << acc_corrigida.x() << ", " << acc_corrigida.y() << ", " << acc_corrigida.z() << "]");

  // q = q * q{ (w_m - w_b)*dt }
  Eigen::Vector3d delta_theta = (w_m - w_b) * dt;
  RCLCPP_DEBUG_STREAM(logger_, "[predict] delta_theta: [" << delta_theta.x() << ", " << delta_theta.y() << ", " << delta_theta.z() << "]");

  if (delta_theta.norm() > 1e-9) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()));
    q = (q * dq).normalized();
  }
  x_nominal_(StateNominal::QW) = q.w();
  x_nominal_(StateNominal::QX) = q.x();
  x_nominal_(StateNominal::QY) = q.y();
  x_nominal_(StateNominal::QZ) = q.z();

  // Biases e gravidade permanecem constantes na predição (Eq. 4, 5, 6)

  // Calculamos Fx (Matriz Jacobiana do erro em relação ao estado)
  RCLCPP_DEBUG_STREAM(logger_, "[predict] x_nominal_ (depois):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(NUM_STATES_ERROR, NUM_STATES_ERROR);

  Fx.block<3, 3>(0, 3)  = Eigen::Matrix3d::Identity() * dt;                     // d_p / d_v
  Fx.block<3, 3>(3, 6)  = -R * skew_symmetric(a_m - a_b) * dt;                  // d_v / d_theta
  Fx.block<3, 3>(3, 9)  = -R * dt;                                              // d_v / d_ba
  Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;                     // d_v / d_g
  Fx.block<3, 3>(6, 6)  = Sophus::SO3d::exp(delta_theta).matrix().transpose();  // d_theta / d_theta
  Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;                    // d_theta / d_bw

  // Matriz de Ruído Qw (Baseada na Eq. 14)
  // Aqui você usa os ganhos r_gains_ e q_gains_ definidos no seu .hpp
  Eigen::Matrix<double, 12, 12> Qw = Eigen::Matrix<double, 12, 12>::Zero();
  Qw.block<3, 3>(0, 0)             = Eigen::Matrix3d::Identity() * std::pow(_default_gains_.velocity_linear, 2) * dt * dt;
  Qw.block<3, 3>(3, 3)             = Eigen::Matrix3d::Identity() * std::pow(_default_gains_.orientation, 2) * dt * dt;
  Qw.block<3, 3>(6, 6)             = Eigen::Matrix3d::Identity() * std::pow(_default_gains_.accelerometer, 2) * dt * dt;  // Variância do bias accel
  Qw.block<3, 3>(9, 9)             = Eigen::Matrix3d::Identity() * std::pow(_default_gains_.gyroscope, 2) * dt * dt;      // Variância do bias gyro

  // Matriz de mapeamento de ruído Fw (Eq. 13)
  Eigen::MatrixXd Fw    = Eigen::MatrixXd::Zero(NUM_STATES_ERROR, 12);
  Fw.block<3, 3>(3, 0)  = Eigen::Matrix3d::Identity();
  Fw.block<3, 3>(6, 3)  = Eigen::Matrix3d::Identity();
  Fw.block<3, 3>(9, 6)  = Eigen::Matrix3d::Identity();
  Fw.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

  // 4. Propagação da Covariância do Erro (Equação 11)
  // P = Fx * P * Fx^T + Fw * Qw * Fw^T
  // Atualiza P
  P_ = Fx * P_ * Fx.transpose() + Fw * Qw * Fw.transpose();
  RCLCPP_DEBUG_STREAM(logger_, "[predict] P_ (trace): " << P_.trace());
}

void ErrorStateEstimator::correct(const MeasurementPackage &measurements) {
  // 1. Árbitro de Outliers (Seção III)
  RCLCPP_DEBUG_STREAM(logger_, "[correct] x_nominal_ (antes):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  RCLCPP_DEBUG_STREAM(logger_, "[correct] delta_x_ (antes):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Angle:     " << delta_x_.segment<3>(StateError::ROLL).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << delta_x_.segment<3>(StateError::DBGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << delta_x_.segment<3>(StateError::DBAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << delta_x_.segment<3>(StateError::DGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 6).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 9).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 12).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 15).transpose());

  if (measurements.openvins != nullptr) {
    // if (outlier_arbiter(*measurements.openvins, SensorIndex::SENSOR_INDEX_VIO)) {
    RCLCPP_DEBUG(logger_, "[correct] Applying OpenVINS correction.");
    apply_odometry_correction(*measurements.openvins, _gains_, SensorIndex::SENSOR_INDEX_VIO);
    // } else {
    // RCLCPP_DEBUG(logger_, "[correct] OpenVINS odometry measurement considered an outlier. Skipping correction.");
    // return;
    // }
  }

  if (measurements.fast_lio != nullptr) {
    // if (outlier_arbiter(*measurements.fast_lio, SensorIndex::SENSOR_INDEX_LIDAR)) {
    RCLCPP_DEBUG(logger_, "[correct] Applying FastLIO correction.");
    apply_odometry_correction(*measurements.fast_lio, _gains_, SensorIndex::SENSOR_INDEX_LIDAR);
    // } else {
    // RCLCPP_DEBUG(logger_, "[correct] FastLIO odometry measurement considered an outlier. Skipping correction.");
    // return;
    // }
  }

  if (measurements.px4_api != nullptr) {
    // if (outlier_arbiter(*measurements.px4_api, SensorIndex::SENSOR_INDEX_PX4)) {
    RCLCPP_DEBUG(logger_, "[correct] Applying PX4 correction.");
    apply_odometry_correction(*measurements.px4_api, _gains_, SensorIndex::SENSOR_INDEX_PX4);
    // } else {
    // RCLCPP_DEBUG(logger_, "[correct] PX4 odometry measurement considered an outlier. Skipping correction.");
    // return;
    // }
  }

  // 3. Injeção e Reset (Seção II-B)
  RCLCPP_DEBUG_STREAM(logger_, "[correct] x_nominal_ (depois):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  RCLCPP_DEBUG_STREAM(logger_, "[correct] delta_x_ (depois):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Angle:     " << delta_x_.segment<3>(StateError::ROLL).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << delta_x_.segment<3>(StateError::DBGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << delta_x_.segment<3>(StateError::DBAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << delta_x_.segment<3>(StateError::DGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 6).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 9).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 12).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 15).transpose());
  inject_error_and_reset();
}


void ErrorStateEstimator::apply_odometry_correction(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains, int sensor_idx) {


  const int sensor_base = StateError::TOTAL_SIZE + 6 * sensor_idx;
  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] sensor_idx=%d, sensor_base=%d, NUM_STATES_ERROR=%d", sensor_idx, sensor_base, NUM_STATES_ERROR);
  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] odom pos: [%f, %f, %f] vel: [%f, %f, %f] quat: [%f, %f, %f, %f]", odom.pose.pose.position.x,
               odom.pose.pose.position.y, odom.pose.pose.position.z, odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
               odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

  if (sensor_base + 6 > NUM_STATES_ERROR) {
    RCLCPP_FATAL(logger_, "MATRIX INDEX OUT OF BOUNDS! Block access at %d exceeds size %d. Check num_sensors_with_drift initialization!", sensor_base,
                 NUM_STATES_ERROR);
    // Continuing will likely crash
  }

  // =========================
  // Medições do sensor
  // =========================
  Eigen::Vector3d    p_mv(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  Eigen::Vector3d    v_mv(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  Eigen::Quaterniond q_mv(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  // =========================
  // Estado nominal UAV
  // =========================
  Eigen::Vector3d    p_uav = x_nominal_.segment<3>(StateNominal::PX);
  Eigen::Vector3d    v_uav = x_nominal_.segment<3>(StateNominal::VX);
  Eigen::Quaterniond q_uav(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] UAV p: [%f, %f, %f] v: [%f, %f, %f] q: [%f, %f, %f, %f]", p_uav.x(), p_uav.y(), p_uav.z(), v_uav.x(),
               v_uav.y(), v_uav.z(), q_uav.w(), q_uav.x(), q_uav.y(), q_uav.z());

  // =========================
  // Drift do sensor i
  // =========================
  Eigen::Vector3d    p_drift = get_sensor_p_drift(sensor_idx);
  Eigen::Quaterniond q_drift = get_sensor_q_drift(sensor_idx);
  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] p_drift: [%f, %f, %f] q_drift: [%f, %f, %f, %f]", p_drift.x(), p_drift.y(), p_drift.z(), q_drift.w(),
               q_drift.x(), q_drift.y(), q_drift.z());

  // ============================================================
  // CORREÇÃO DE POSIÇÃO
  // h(x) = R{q_i} p + p_i
  // ============================================================
  Eigen::Vector3d p_pred = q_drift.toRotationMatrix() * p_uav + p_drift;
  Eigen::Vector3d y_p    = p_mv - p_pred;
  Eigen::MatrixXd Hp     = Eigen::MatrixXd::Zero(3, NUM_STATES_ERROR);

  // δp
  Hp.block<3, 3>(0, StateError::PX) = q_drift.toRotationMatrix();

  // δp_i
  Hp.block<3, 3>(0, sensor_base) = Eigen::Matrix3d::Identity();

  // ============================================================
  // CORREÇÃO DE ORIENTAÇÃO
  // e_theta = log( Rᵀ{q_i q} R{q_mv} )
  // ============================================================
  Eigen::Quaterniond q_pred = q_drift * q_uav;
  Eigen::Matrix3d    R_err  = q_pred.toRotationMatrix().transpose() * q_mv.toRotationMatrix();
  Eigen::AngleAxisd  aa(R_err);
  Eigen::Vector3d    y_theta;
  double             angle = aa.angle();
  if (angle < 1e-6) {
    // Região linear do log-map (erro pequeno)
    y_theta.setZero();
  } else {
    // log(R)^vee = axis * angle
    y_theta = aa.axis() * angle;
  }
  Eigen::MatrixXd Htheta = Eigen::MatrixXd::Zero(3, NUM_STATES_ERROR);

  // δθ do UAV
  Htheta.block<3, 3>(0, StateError::ROLL) = Eigen::Matrix3d::Identity();

  // δθ_i do sensor (IMPORTANTE: Rᵀ{q_mv})
  Htheta.block<3, 3>(0, sensor_base + 3) = q_mv.toRotationMatrix().transpose();


  // ============================================================
  // CORREÇÃO DE VELOCIDADE LINEAR (frame do robô)
  // h(x) = Rᵀ(q) v
  // ============================================================

  // Predição da medição
  Eigen::Matrix3d R      = q_uav.toRotationMatrix();
  Eigen::Vector3d v_pred = R.transpose() * v_uav;
  // Inovação
  Eigen::Vector3d y_v = v_mv - v_pred;

  // Jacobiano
  Eigen::MatrixXd Hv = Eigen::MatrixXd::Zero(3, NUM_STATES_ERROR);

  // δv (velocidade no mundo)
  Hv.block<3, 3>(0, 3) = R.transpose();

  // δθ (erro de orientação)
  Hv.block<3, 3>(0, 6) = -R.transpose() * skew_symmetric(v_uav);

  // ============================================================
  // FUSÃO
  // ============================================================
  Eigen::VectorXd y(9);
  y << y_p, y_v, y_theta;
  Eigen::MatrixXd H                  = Eigen::MatrixXd::Zero(9, NUM_STATES_ERROR);
  H.block(0, 0, 3, NUM_STATES_ERROR) = Hp;
  H.block(3, 0, 3, NUM_STATES_ERROR) = Hv;
  H.block(6, 0, 3, NUM_STATES_ERROR) = Htheta;
  Eigen::MatrixXd V                  = Eigen::MatrixXd::Zero(9, 9);
  V.block<3, 3>(0, 0)                = gains.odom[sensor_idx].position * gains.odom[sensor_idx].position * Eigen::Matrix3d::Identity();
  V.block<3, 3>(3, 3)                = gains.odom[sensor_idx].velocity_linear * gains.odom[sensor_idx].velocity_linear * Eigen::Matrix3d::Identity();
  V.block<3, 3>(6, 6)                = gains.odom[sensor_idx].orientation * gains.odom[sensor_idx].orientation * Eigen::Matrix3d::Identity();
  RCLCPP_DEBUG_STREAM(logger_, "[apply_odometry_correction] y:");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << y.segment<3>(0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << y.segment<3>(3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ Angle:    " << y.segment<3>(6).transpose());

  // Eigen::VectorXd y(6);
  // y << y_p, y_theta;
  // Eigen::MatrixXd H                  = Eigen::MatrixXd::Zero(6, NUM_STATES_ERROR);
  // H.block(0, 0, 3, NUM_STATES_ERROR) = Hp;
  // H.block(3, 0, 3, NUM_STATES_ERROR) = Htheta;
  // Eigen::MatrixXd V                  = Eigen::MatrixXd::Zero(6, 6);
  // V.block<3, 3>(0, 0)                = gains.odom[sensor_idx].position * gains.odom[sensor_idx].position * Eigen::Matrix3d::Identity();
  // V.block<3, 3>(3, 3)                = gains.odom[sensor_idx].orientation * gains.odom[sensor_idx].orientation * Eigen::Matrix3d::Identity();
  // RCLCPP_DEBUG_STREAM(logger_, "[apply_odometry_correction] y:");
  // RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << y.segment<3>(0).transpose());
  // RCLCPP_DEBUG_STREAM(logger_, " └ Angle:    " << y.segment<3>(3).transpose());

  Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();

  // Note o uso de _STREAM e dos operadores <<

  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] K (trace): %f", K.trace());

  if (delta_x_.size() == (K * y).size()) {
    delta_x_ += K * y;
  } else {
    RCLCPP_ERROR_STREAM(
        logger_, "[apply_odometry_correction] ERRO: delta_x_ size (" << delta_x_.size() << ") != K*y size (" << (K * y).size() << "). Correção ignorada!");
  }

  P_ = (Eigen::MatrixXd::Identity(NUM_STATES_ERROR, NUM_STATES_ERROR) - K * H) * P_;
  RCLCPP_DEBUG(logger_, "[apply_odometry_correction] P_ (trace): %f", P_.trace());
}


// Getters conforme equações (22)-(28) do artigo
Eigen::Vector3d ErrorStateEstimator::get_position() const {
  // Retorna p_t = p + delta_p (após o reset, p contém a estimativa corrigida)
  return x_nominal_.segment<3>(StateNominal::PX);
}

Eigen::Quaterniond ErrorStateEstimator::get_orientation() const {
  // Retorna q_t = q * dq (após o reset, q contém a orientação corrigida)
  Eigen::Quaterniond q =
      Eigen::Quaterniond(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  q.normalize();
  return q;
}

Eigen::Vector3d ErrorStateEstimator::get_linear_velocity() const {
  // Retorna v_t = v + delta_v
  return x_nominal_.segment<3>(StateNominal::VX);
}

Eigen::Vector3d ErrorStateEstimator::get_angular_velocity() const {
  // A velocidade angular verdadeira é a medição menos o bias estimado: w_t = w_mv - w_b
  Eigen::Vector3d w_mv(last_imu_measure_.angular_velocity.x, last_imu_measure_.angular_velocity.y, last_imu_measure_.angular_velocity.z);
  Eigen::Vector3d w_b = x_nominal_.segment<3>(StateNominal::BGX);
  return w_mv - w_b;
}

nav_msgs::msg::Odometry ErrorStateEstimator::get_odometry() const {
  nav_msgs::msg::Odometry odom;

  // Pose: Posição (Eq. 22) e Orientação (Eq. 24)
  odom.pose.pose.position.x = x_nominal_(StateNominal::PX);
  odom.pose.pose.position.y = x_nominal_(StateNominal::PY);
  odom.pose.pose.position.z = x_nominal_(StateNominal::PZ);

  odom.pose.pose.orientation.w = x_nominal_(StateNominal::QW);
  odom.pose.pose.orientation.x = x_nominal_(StateNominal::QX);
  odom.pose.pose.orientation.y = x_nominal_(StateNominal::QY);
  odom.pose.pose.orientation.z = x_nominal_(StateNominal::QZ);

  // Twist Linear (Eq. 23)
  odom.twist.twist.linear.x = x_nominal_(StateNominal::VX);
  odom.twist.twist.linear.y = x_nominal_(StateNominal::VY);
  odom.twist.twist.linear.z = x_nominal_(StateNominal::VZ);

  // Twist Angular: Medição da IMU corrigida pelo bias estimado
  Eigen::Vector3d w_m(last_imu_measure_.angular_velocity.x, last_imu_measure_.angular_velocity.y, last_imu_measure_.angular_velocity.z);
  Eigen::Vector3d w_b         = x_nominal_.segment<3>(StateNominal::BGX);
  Eigen::Vector3d w_corrigida = w_m - w_b;

  odom.twist.twist.angular.x = w_corrigida.x();
  odom.twist.twist.angular.y = w_corrigida.y();
  odom.twist.twist.angular.z = w_corrigida.z();

  // Covariância da Pose (6x6) baseada na matriz P do filtro
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      int idx_i                       = (i < 3) ? StateError::PX + i : StateError::ROLL + (i - 3);
      int idx_j                       = (j < 3) ? StateError::PX + j : StateError::ROLL + (j - 3);
      odom.pose.covariance[i * 6 + j] = P_(idx_i, idx_j);
    }
  }

  return odom;
}

/**
 * @brief Injeta o erro estimado no estado nominal e reseta delta_x.
 * Segue as equações (22)-(28).
 */
void ErrorStateEstimator::inject_error_and_reset() {
  RCLCPP_DEBUG_STREAM(logger_, "[inject_error_and_reset] x_nominal_ (antes):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  RCLCPP_DEBUG_STREAM(logger_, "[inject_error_and_reset] delta_x_ (antes):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Angle:     " << delta_x_.segment<3>(StateError::ROLL).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << delta_x_.segment<3>(StateError::DBGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << delta_x_.segment<3>(StateError::DBAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << delta_x_.segment<3>(StateError::DGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 6).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 9).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 12).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 15).transpose());

  // 1. Posição e Velocidade (Soma direta: p = p + delta_p)
  x_nominal_.segment<3>(StateNominal::PX) += delta_x_.segment<3>(StateError::PX);
  x_nominal_.segment<3>(StateNominal::VX) += delta_x_.segment<3>(StateError::VX);

  // 2. Orientação (Composição de Quatérnios: q = q * q{delta_theta})
  // O artigo trata delta_theta como um vetor de erro no espaço tangente.
  Eigen::Vector3d delta_theta = delta_x_.segment<3>(StateError::ROLL);

  if (delta_theta.norm() > 1e-9) {
    Eigen::Quaterniond q_nominal(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
    // Converte delta_theta (ângulo-eixo) para quatérnio de erro
    Eigen::Quaterniond dq(Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()));
    // Atualiza a orientação nominal (Eq. 24)
    q_nominal                    = (q_nominal * dq).normalized();
    x_nominal_(StateNominal::QW) = q_nominal.w();
    x_nominal_(StateNominal::QX) = q_nominal.x();
    x_nominal_(StateNominal::QY) = q_nominal.y();
    x_nominal_(StateNominal::QZ) = q_nominal.z();
    // RCLCPP_DEBUG_STREAM(logger_, "[inject_error_and_reset] delta_theta: [" << delta_theta.x() << ", " << delta_theta.y() << ", " << delta_theta.z() << "]");
  }

  // 3. Biases e Gravidade (Soma direta)
  x_nominal_.segment<3>(StateNominal::BAX) += delta_x_.segment<3>(StateError::DBAX);
  x_nominal_.segment<3>(StateNominal::BGX) += delta_x_.segment<3>(StateError::DBGX);
  x_nominal_.segment<3>(StateNominal::GX) += delta_x_.segment<3>(StateError::DGX);

  x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0) += delta_x_.segment<3>(StateError::TOTAL_SIZE + 0);  // Drift Posição LIO
  Eigen::Vector3d delta_theta_lio = delta_x_.segment<3>(StateError::TOTAL_SIZE + 3);                       // Drift Ângulo LIO
  if (delta_theta_lio.norm() > 1e-9) {
    Eigen::Quaterniond q_drift_lio(x_nominal_(StateNominal::TOTAL_SIZE + 3), x_nominal_(StateNominal::TOTAL_SIZE + 4), x_nominal_(StateNominal::TOTAL_SIZE + 5),
                                   x_nominal_(StateNominal::TOTAL_SIZE + 6));

    Eigen::Quaterniond dq_lio(Eigen::AngleAxisd(delta_theta_lio.norm(), delta_theta_lio.normalized()));
    q_drift_lio                              = (q_drift_lio * dq_lio).normalized();
    x_nominal_(StateNominal::TOTAL_SIZE + 3) = q_drift_lio.w();
    x_nominal_(StateNominal::TOTAL_SIZE + 4) = q_drift_lio.x();
    x_nominal_(StateNominal::TOTAL_SIZE + 5) = q_drift_lio.y();
    x_nominal_(StateNominal::TOTAL_SIZE + 6) = q_drift_lio.z();
  }

  x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7) += delta_x_.segment<3>(StateError::TOTAL_SIZE + 6);  // Drift Posição VIO
  Eigen::Vector3d delta_theta_vio = delta_x_.segment<3>(StateError::TOTAL_SIZE + 9);                       // Drift Ângulo VIO
  if (delta_theta_vio.norm() > 1e-9) {
    Eigen::Quaterniond q_drift_vio(x_nominal_(StateNominal::TOTAL_SIZE + 10), x_nominal_(StateNominal::TOTAL_SIZE + 11),
                                   x_nominal_(StateNominal::TOTAL_SIZE + 12), x_nominal_(StateNominal::TOTAL_SIZE + 13));
    Eigen::Quaterniond dq_vio(Eigen::AngleAxisd(delta_theta_vio.norm(), delta_theta_vio.normalized()));
    q_drift_vio                               = (q_drift_vio * dq_vio).normalized();
    x_nominal_(StateNominal::TOTAL_SIZE + 10) = q_drift_vio.w();
    x_nominal_(StateNominal::TOTAL_SIZE + 11) = q_drift_vio.x();
    x_nominal_(StateNominal::TOTAL_SIZE + 12) = q_drift_vio.y();
    x_nominal_(StateNominal::TOTAL_SIZE + 13) = q_drift_vio.z();
  }

  x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14) += delta_x_.segment<3>(StateError::TOTAL_SIZE + 12);  // Drift Posição PX4
  Eigen::Vector3d delta_theta_px4 = delta_x_.segment<3>(StateError::TOTAL_SIZE + 15);                        // Drift Ângulo PX4
  if (delta_theta_px4.norm() > 1e-9) {
    Eigen::Quaterniond q_drift_px4(x_nominal_(StateNominal::TOTAL_SIZE + 17), x_nominal_(StateNominal::TOTAL_SIZE + 18),
                                   x_nominal_(StateNominal::TOTAL_SIZE + 19), x_nominal_(StateNominal::TOTAL_SIZE + 20));
    Eigen::Quaterniond dq_px4(Eigen::AngleAxisd(delta_theta_px4.norm(), delta_theta_px4.normalized()));
    q_drift_px4                               = (q_drift_px4 * dq_px4).normalized();
    x_nominal_(StateNominal::TOTAL_SIZE + 17) = q_drift_px4.w();
    x_nominal_(StateNominal::TOTAL_SIZE + 18) = q_drift_px4.x();
    x_nominal_(StateNominal::TOTAL_SIZE + 19) = q_drift_px4.y();
    x_nominal_(StateNominal::TOTAL_SIZE + 20) = q_drift_px4.z();
  }

  // 4. Reset do estado de erro (Seção II-B: "The error state is reset to zero")
  delta_x_.setZero();

  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(NUM_STATES_ERROR, NUM_STATES_ERROR);

  // Correção da covariância para a orientação do Robô (Índice 6 a 8)
  G.block<3, 3>(6, 6) -= skew_symmetric(delta_theta);

  // LIO Drift Angle (Offset + 3)
  G.block<3, 3>(StateError::TOTAL_SIZE + 3, StateError::TOTAL_SIZE + 3) -= skew_symmetric(delta_theta_lio);
  // VIO Drift Angle (Offset + 9)
  G.block<3, 3>(StateError::TOTAL_SIZE + 9, StateError::TOTAL_SIZE + 9) -= skew_symmetric(delta_theta_vio);
  // PX4 Drift Angle (Offset + 15)
  G.block<3, 3>(StateError::TOTAL_SIZE + 15, StateError::TOTAL_SIZE + 15) -= skew_symmetric(delta_theta_px4);

  P_ = G * P_ * G.transpose();

  RCLCPP_DEBUG_STREAM(logger_, "[inject_error_and_reset] x_nominal_ (depois):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << x_nominal_.segment<3>(StateNominal::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Quat:     " << x_nominal_.segment<4>(StateNominal::QW).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << x_nominal_.segment<3>(StateNominal::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << x_nominal_.segment<3>(StateNominal::BGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << x_nominal_.segment<3>(StateNominal::BAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << x_nominal_.segment<3>(StateNominal::GX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 7).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 10).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << x_nominal_.segment<3>(StateNominal::TOTAL_SIZE + 14).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << x_nominal_.segment<4>(StateNominal::TOTAL_SIZE + 17).transpose());

  RCLCPP_DEBUG_STREAM(logger_, "[inject_error_and_reset] delta_x_ (depois):");
  RCLCPP_DEBUG_STREAM(logger_, " ├ Pos:      " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Angle:     " << delta_x_.segment<3>(StateError::ROLL).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Lin. Vel: " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Gyro: " << delta_x_.segment<3>(StateError::DBGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Bias Acc: " << delta_x_.segment<3>(StateError::DBAX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ Gravidade: " << delta_x_.segment<3>(StateError::DGX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 0).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [LIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 3).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 6).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [VIO] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 9).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " ├ [PX4] Drift Pos: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 12).transpose());
  RCLCPP_DEBUG_STREAM(logger_, " └ [PX4] Drift Angle: " << delta_x_.segment<3>(StateError::TOTAL_SIZE + 15).transpose());
}

Eigen::Vector3d ErrorStateEstimator::get_sensor_p_drift(int sensor_idx) const {
  // Retorna o drift de posição p_i do sensor específico
  // O índice depende de como você organizou seu x_nominal estendido
  int offset = StateNominal::TOTAL_SIZE + (sensor_idx * DRIFT_SIZE);
  return x_nominal_.segment<3>(offset);
}

Eigen::Quaterniond ErrorStateEstimator::get_sensor_q_drift(int sensor_idx) const {
  // Retorna o drift de orientação q_i do sensor específico
  int offset = StateNominal::TOTAL_SIZE + (sensor_idx * DRIFT_SIZE) + 3;
  return Eigen::Quaterniond(x_nominal_(offset), x_nominal_(offset + 1), x_nominal_(offset + 2), x_nominal_(offset + 3));
}

bool ErrorStateEstimator::outlier_arbiter(const nav_msgs::msg::Odometry &measure, const int &sensor_type) {
  // --------------------------------------------------
  // 1. Verifica se há limites configurados
  // --------------------------------------------------

  std::string sensor_name = "";
  if (_limits_.find(sensor_type) == _limits_.end()) {
    sensor_name = (sensor_type == SensorIndex::SENSOR_INDEX_LIDAR) ? "LIDAR"
                  : (sensor_type == SensorIndex::SENSOR_INDEX_VIO) ? "VIO"
                  : (sensor_type == SensorIndex::SENSOR_INDEX_PX4) ? "PX4"
                                                                   : "UNKNOWN";

    RCLCPP_WARN(logger_, "Arbiter: sensor [%s] sem limites definidos. Rejeitando.", sensor_name.c_str());
    return false;
  }

  const auto &limits = _limits_.at(sensor_type);


  // --------------------------------------------------
  // 2. Estado nominal atual (UAV)
  // --------------------------------------------------
  Eigen::Vector3d p_uav = x_nominal_.segment<3>(StateNominal::PX);

  Eigen::Quaterniond q_uav(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  q_uav.normalize();

  // --------------------------------------------------
  // 3. Medição de posição
  // --------------------------------------------------
  Eigen::Vector3d p_meas(measure.pose.pose.position.x, measure.pose.pose.position.y, measure.pose.pose.position.z);
  Eigen::Vector3d pos_innovation = p_meas - p_uav;

  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: UAV Position: " << p_uav.transpose());
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Measured Position: " << p_meas.transpose());
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: UAV Orientation (quat): " << q_uav.w() << ", " << q_uav.x() << ", " << q_uav.y() << ", " << q_uav.z());
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Measured Orientation (quat): " << measure.pose.pose.orientation.w << ", " << measure.pose.pose.orientation.x << ", "
                                                                        << measure.pose.pose.orientation.y << ", " << measure.pose.pose.orientation.z);
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Position Innovation: " << pos_innovation.transpose());
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Position x limit: " << limits.pos.x() << ", Innovation: " << std::abs(pos_innovation.x()));
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Position y limit: " << limits.pos.y() << ", Innovation: " << std::abs(pos_innovation.y()));
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Position z limit: " << limits.pos.z() << ", Innovation: " << std::abs(pos_innovation.z()));

  if (std::abs(pos_innovation.x()) > limits.pos.x())
    return false;
  if (std::abs(pos_innovation.y()) > limits.pos.y())
    return false;
  if (std::abs(pos_innovation.z()) > limits.pos.z())
    return false;

  // --------------------------------------------------
  // 4. Medição de orientação (ES-EKF correto)
  // --------------------------------------------------
  Eigen::Quaterniond q_meas(measure.pose.pose.orientation.w, measure.pose.pose.orientation.x, measure.pose.pose.orientation.y, measure.pose.pose.orientation.z);
  q_meas.normalize();

  // Erro de orientação: δq = q̂⁻¹ ⊗ q_meas
  Eigen::Quaterniond q_err = q_uav.conjugate() * q_meas;
  q_err.normalize();

  Eigen::AngleAxisd aa(q_err);
  Eigen::Vector3d   rot_innovation = aa.angle() * aa.axis();

  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Rotation Innovation: " << rot_innovation.transpose());
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Rotation x limit: " << limits.rot.x() << ", Innovation: " << std::abs(rot_innovation.x()));
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Rotation y limit: " << limits.rot.y() << ", Innovation: " << std::abs(rot_innovation.y()));
  RCLCPP_DEBUG_STREAM(logger_, "Arbiter: Rotation z limit: " << limits.rot.z() << ", Innovation: " << std::abs(rot_innovation.z()));

  if (std::abs(rot_innovation.x()) > limits.rot.x())
    return false;
  if (std::abs(rot_innovation.y()) > limits.rot.y())
    return false;
  if (std::abs(rot_innovation.z()) > limits.rot.z())
    return false;

  // --------------------------------------------------
  // 5. Medição aprovada
  // --------------------------------------------------
  return true;
}

Eigen::Matrix3d ErrorStateEstimator::skew_symmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

/* set_verbosity() //{ */
void ErrorStateEstimator::set_verbosity(const std::string &verbosity) {
  if (verbosity == "SILENT") {
    logger_.set_level(rclcpp::Logger::Level::Fatal);
  } else if (verbosity == "ERROR") {
    logger_.set_level(rclcpp::Logger::Level::Error);
  } else if (verbosity == "WARNING") {
    logger_.set_level(rclcpp::Logger::Level::Warn);
  } else if (verbosity == "DEBUG") {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  } else {
    logger_.set_level(rclcpp::Logger::Level::Info);
  }

  RCLCPP_INFO_STREAM(logger_, "Verbosity level set to: " << verbosity);
}
//}

}  // namespace laser_uav_estimators
