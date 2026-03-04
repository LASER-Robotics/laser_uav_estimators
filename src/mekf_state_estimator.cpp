#include <laser_uav_estimators/mekf_state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace laser_uav_estimators
{

/* MEKFEstimator() //{ */
MEKFEstimator::MEKFEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const MeasurementNoiseGains &gains,
                             const NoiseGains &default_gains, const std::string &verbosity)
    : _gains_(gains),
      _default_gains_(default_gains),
      _mass_(mass),
      _allocation_matrix_(allocation_matrix),
      _inertia_(inertia),
      logger_(rclcpp::get_logger("mekf_state_estimator")) {
  set_verbosity(verbosity);

  if (verbosity == "DEBUG") {
    is_debug_ = true;
  }

  RCLCPP_INFO(logger_, "--- MEKF STATE ESTIMATOR CONSTRUCTOR ---");

  x_nominal_                   = Eigen::VectorXd::Zero(13);  // Posição (3), Velocidade Linear (3), Orientação (4), Velocidade Angular (3)
  x_nominal_(StateNominal::QW) = 1.0;                        // Inicializa a orientação como identidade


  delta_x_ = Eigen::VectorXd::Zero(12);                // Erros em Posição (3), Velocidade Linear (3), Orientação (3), Velocidade Angular (3)
  P_       = Eigen::MatrixXd::Identity(12, 12) * 0.1;  // Covariância inicial pequena
}

void MEKFEstimator::predict(const Eigen::VectorXd &u, double dt) {
  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0);
    double          rad2deg = 180.0 / M_PI;

    RCLCPP_DEBUG_STREAM(logger_, "--- [BEFORE] - MEKF Prediction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[2] * rad2deg << " P:" << euler[1] * rad2deg << " Y:" << euler[0] * rad2deg);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
    RCLCPP_DEBUG_STREAM(logger_, "--- - Entrada:  ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ u:          " << u.transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ dt:         " << dt);
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
  }

  Eigen::Vector3d    position         = x_nominal_.segment<3>(StateNominal::PX);
  Eigen::Vector3d    linear_velocity  = x_nominal_.segment<3>(StateNominal::VX);
  Eigen::Vector3d    angular_velocity = x_nominal_.segment<3>(StateNominal::WX);
  Eigen::Quaterniond orientation =
      Eigen::Quaterniond(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  Eigen::Matrix3d R_body_to_inertial = orientation.toRotationMatrix();

  Eigen::Matrix<double, 4, 1> force_body = _allocation_matrix_ * u;
  double                      thrust     = force_body(0);
  Eigen::Vector3d             tau        = force_body.segment<3>(1);
  Eigen::Vector3d             bz         = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d             ez         = Eigen::Vector3d(0.0, 0.0, 1.0);

  Eigen::Vector3d aW = R_body_to_inertial * ((thrust / _mass_) * bz) + ((-GRAVITY) * ez);

  Eigen::Vector3d position_predict = position + (dt * R_body_to_inertial * linear_velocity) + ((0.5 * dt * dt) * aW);

  Eigen::Vector3d linear_velocity_predict =
      linear_velocity + dt * (((thrust / _mass_) * bz) + (R_body_to_inertial.transpose() * (-GRAVITY) * ez) - angular_velocity.cross(linear_velocity));

  // Propagação correta da orientação usando ExpSO3Quaternion
  Eigen::Vector3d    half_theta_vec      = dt * angular_velocity;
  Eigen::Quaterniond delta_q             = ExpSO3Quaternion(half_theta_vec);
  Eigen::Quaterniond orientation_predict = orientation * delta_q;
  orientation_predict.normalize();

  Eigen::Vector3d angular_velocity_predict = angular_velocity + (dt * (_inertia_.inverse() * (tau - angular_velocity.cross(_inertia_ * angular_velocity))));


  x_nominal_predict                   = Eigen::VectorXd::Zero(13);
  x_nominal_predict(StateNominal::QW) = 1.0;

  x_nominal_predict.segment<3>(StateNominal::PX) = position_predict;
  x_nominal_predict.segment<3>(StateNominal::VX) = linear_velocity_predict;
  x_nominal_predict.segment<4>(StateNominal::QW) =
      Eigen::Vector4d(orientation_predict.w(), orientation_predict.x(), orientation_predict.y(), orientation_predict.z());
  x_nominal_predict.segment<3>(StateNominal::WX) = angular_velocity_predict;


  Eigen::MatrixXd Fx                             = Eigen::MatrixXd::Identity(12, 12);
  Fx.block<3, 3>(StateError::PX, StateError::VX) = dt * R_body_to_inertial;
  Fx.block<3, 3>(StateError::PX, StateError::ROLL) =
      -dt * R_body_to_inertial * skew_symmetric(linear_velocity) - (0.5 * dt * dt * R_body_to_inertial * skew_symmetric((thrust / _mass_) * bz));

  Fx.block<3, 3>(StateError::VX, StateError::VX)   = Eigen::Matrix3d::Identity() - dt * skew_symmetric(angular_velocity);
  Fx.block<3, 3>(StateError::VX, StateError::ROLL) = -dt * skew_symmetric(R_body_to_inertial.transpose() * (-GRAVITY) * ez);
  Fx.block<3, 3>(StateError::VX, StateError::WX)   = -dt * skew_symmetric(linear_velocity);

  Fx.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() - dt * skew_symmetric(angular_velocity);
  Fx.block<3, 3>(StateError::ROLL, StateError::WX)   = dt * Eigen::Matrix3d::Identity();

  Fx.block<3, 3>(StateError::WX, StateError::WX) =
      Eigen::Matrix3d::Identity() - (dt * _inertia_.inverse() * (skew_symmetric(angular_velocity) * _inertia_ - skew_symmetric(_inertia_ * angular_velocity)));

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);

  Q.block<2, 2>(StateError::PX, StateError::PX) = Eigen::Matrix2d::Identity() * _default_gains_.position_xy;
  Q(StateError::PZ, StateError::PZ)             = 1.0 * _default_gains_.position_z;

  Q.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() * _default_gains_.orientation;

  Q.block<2, 2>(StateError::VX, StateError::VX) = Eigen::Matrix2d::Identity() * _default_gains_.velocity_linear_xy;
  Q(StateError::VZ, StateError::VZ)             = 1.0 * _default_gains_.velocity_linear_z;

  Q.block<3, 3>(StateError::WX, StateError::WX) = Eigen::Matrix3d::Identity() * _default_gains_.velocity_angular;


  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_predict(StateNominal::QW), x_nominal_predict(StateNominal::QX), x_nominal_predict(StateNominal::QY),
                               x_nominal_predict(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0);
    double          rad2deg = 180.0 / M_PI;

    RCLCPP_DEBUG_STREAM(logger_, "--- [AFTER] - MEKF Prediction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_predict.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_predict.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[2] * rad2deg << " P:" << euler[1] * rad2deg << " Y:" << euler[0] * rad2deg);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_predict.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_predict.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());
  }

  P_ = Fx * P_ * Fx.transpose() + Q;
}

void MEKFEstimator::correct(const MeasurementPackage &measurements) {

  bool            has_odom  = (measurements.odometry != nullptr);
  bool            has_range = (measurements.garmin != nullptr);
  Eigen::MatrixXd H;
  Eigen::MatrixXd R;
  Eigen::VectorXd y;

  if (!has_odom && !has_range) {
    RCLCPP_ERROR_STREAM(logger_, "No measurements provided for correction update.");
    return;
  }
  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler = q_final.toRotationMatrix().eulerAngles(0, 1, 2);

    RCLCPP_DEBUG_STREAM(logger_, "--- [BEFORE] - MEKF Correction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[0] << " P:" << euler[1] << " Y:" << euler[2]);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
  }

  if (has_odom) {
    Eigen::Vector3d p_meas =
        Eigen::Vector3d(measurements.odometry->pose.pose.position.x, measurements.odometry->pose.pose.position.y, measurements.odometry->pose.pose.position.z);
    Eigen::Quaterniond q_meas = Eigen::Quaterniond(measurements.odometry->pose.pose.orientation.w, measurements.odometry->pose.pose.orientation.x,
                                                   measurements.odometry->pose.pose.orientation.y, measurements.odometry->pose.pose.orientation.z);
    Eigen::Vector3d    v_meas =
        Eigen::Vector3d(measurements.odometry->twist.twist.linear.x, measurements.odometry->twist.twist.linear.y, measurements.odometry->twist.twist.linear.z);
    Eigen::Vector3d w_meas = Eigen::Vector3d(measurements.odometry->twist.twist.angular.x, measurements.odometry->twist.twist.angular.y,
                                             measurements.odometry->twist.twist.angular.z);

    Eigen::Vector3d euler_mean = q_meas.toRotationMatrix().eulerAngles(0, 1, 2);

    RCLCPP_DEBUG_STREAM(logger_, "Performing correction update with odometry measurements.");
    RCLCPP_DEBUG_STREAM(logger_, "--- Odometry Measurement ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << measurements.odometry->pose.pose.position.x << " " << measurements.odometry->pose.pose.position.y
                                                         << " " << measurements.odometry->pose.pose.position.z);
    RCLCPP_DEBUG_STREAM(
        logger_, "     ├ Quat (w,x,y,z): " << measurements.odometry->pose.pose.orientation.w << " " << measurements.odometry->pose.pose.orientation.x << " "
                                           << measurements.odometry->pose.pose.orientation.y << " " << measurements.odometry->pose.pose.orientation.z);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler_mean[0] << " P:" << euler_mean[1] << " Y:" << euler_mean[2]);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << measurements.odometry->twist.twist.linear.x << " " << measurements.odometry->twist.twist.linear.y
                                                         << " " << measurements.odometry->twist.twist.linear.z);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << measurements.odometry->twist.twist.angular.x << " " << measurements.odometry->twist.twist.angular.y
                                                         << " " << measurements.odometry->twist.twist.angular.z);
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");


    Eigen::VectorXd z(13);
    z.segment<3>(StateNominal::PX) = p_meas;
    z.segment<3>(StateNominal::VX) = v_meas;
    z.segment<4>(StateNominal::QW) = Eigen::Vector4d(q_meas.w(), q_meas.x(), q_meas.y(), q_meas.z());
    z.segment<3>(StateNominal::WX) = w_meas;

    Eigen::VectorXd h(13);
    h.segment<3>(StateNominal::PX) = x_nominal_predict.segment<3>(StateNominal::PX);
    h.segment<3>(StateNominal::VX) = x_nominal_predict.segment<3>(StateNominal::VX);
    h.segment<4>(StateNominal::QW) = x_nominal_predict.segment<4>(StateNominal::QW);
    h.segment<3>(StateNominal::WX) = x_nominal_predict.segment<3>(StateNominal::WX);

    int size_y = y.size();
    y.conservativeResize(size_y + 12);

    y.segment<3>(size_y + StateError::PX) = z.segment<3>(StateNominal::PX) - h.segment<3>(StateNominal::PX);
    y.segment<3>(size_y + StateError::VX) = z.segment<3>(StateNominal::VX) - h.segment<3>(StateNominal::VX);
    y.segment<3>(size_y + StateError::WX) = z.segment<3>(StateNominal::WX) - h.segment<3>(StateNominal::WX);

    // Calcular o quaternion erro
    Eigen::Quaterniond q_hat(h(StateNominal::QW), h(StateNominal::QX), h(StateNominal::QY), h(StateNominal::QZ));

    Eigen::Quaterniond dq = q_hat.inverse() * q_meas;

    // CRÍTICO: Verificar sinal da componente escalar para evitar ambiguidade
    // Quaternions q e -q representam a mesma rotação, mas escolhemos w >= 0
    if (dq.w() < 0.0) {
      dq.w() = -dq.w();
      dq.x() = -dq.x();
      dq.y() = -dq.y();
      dq.z() = -dq.z();
    }

    // Extrair o vetor de erro (aproximação de pequeno ângulo)
    // dθ ≈ 2 * [x, y, z]^T da parte vetorial do quaternion
    y.segment<3>(size_y + StateError::ROLL) = 2.0 * dq.vec();

    int size_H = H.rows();
    H.conservativeResize(size_H + 12, 12);
    H.block(size_H, 0, 12, 12) = Eigen::MatrixXd::Identity(12, 12);

    int size_R = R.rows();
    R.conservativeResize(size_R + 12, size_R + 12);
    R.block(size_R, size_R, 12, 12) = Eigen::MatrixXd::Identity(12, 12);

    R.block<2, 2>(size_R + StateError::PX, size_R + StateError::PX)     = Eigen::Matrix2d::Identity() * _gains_.odometry.position_xy;
    R(size_R + StateError::PZ, size_R + StateError::PZ)                 = 1.0 * _gains_.odometry.position_z;
    R.block<3, 3>(size_R + StateError::ROLL, size_R + StateError::ROLL) = Eigen::Matrix3d::Identity() * _gains_.odometry.orientation;
    R.block<2, 2>(size_R + StateError::VX, size_R + StateError::VX)     = Eigen::Matrix2d::Identity() * _gains_.odometry.velocity_linear_xy;
    R(size_R + StateError::VZ, size_R + StateError::VZ)                 = 1.0 * _gains_.odometry.velocity_linear_z;
    R.block<3, 3>(size_R + StateError::WX, size_R + StateError::WX)     = Eigen::Matrix3d::Identity() * _gains_.odometry.velocity_angular;
  }

  if (has_range) {
    RCLCPP_DEBUG_STREAM(logger_, "Performing correction update with range measurements.");
    RCLCPP_DEBUG_STREAM(logger_, "--- Garmin Range Measurement ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Range:        " << measurements.garmin->range);
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");

    Eigen::VectorXd z(1);
    z(0) = measurements.garmin->range;

    Eigen::VectorXd h(1);
    h(0) = x_nominal_predict(StateNominal::PZ);  // Assuming range is measured along the z-axis

    int size_y = y.size();
    y.conservativeResize(size_y + 1);
    y(size_y) = z(0) - h(0);

    int size_H = H.rows();
    H.conservativeResize(size_H + 1, 12);
    H.row(size_H).setZero();
    H(size_H, StateError::PZ) = 1.0;

    int size_R = R.rows();
    R.conservativeResize(size_R + 1, size_R + 1);
    R.row(size_R).setZero();
    R.col(size_R).setZero();
    R(size_R, size_R) = _gains_.garmin.position_z;
  }

  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  // Verificação de segurança antes de imprimir os segmentos de y
  if (y.size() == 12) {
    RCLCPP_DEBUG_STREAM(logger_, "Innovation y (Full):");
    RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << y.segment<3>(StateError::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << y.segment<3>(StateError::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << y.segment<3>(StateError::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << y.segment<3>(StateError::ROLL).transpose());
  } else if (y.size() == 13) {
    RCLCPP_DEBUG_STREAM(logger_, "Innovation y (Full):");
    RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << y.segment<3>(StateError::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << y.segment<3>(StateError::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << y.segment<3>(StateError::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << y.segment<3>(StateError::ROLL).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └  range:   " << y(12));
  } else {
    RCLCPP_DEBUG_STREAM(logger_, "Innovation y (Partial): " << y.transpose());
  }

  delta_x_ = K * y;

  RCLCPP_DEBUG_STREAM(logger_, "State Correction delta_x:");
  RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << delta_x_.segment<3>(StateError::WX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << delta_x_.segment<3>(StateError::ROLL).transpose());

  P_                                      = (Eigen::MatrixXd::Identity(12, 12) - K * H) * P_;
  x_nominal_.segment<3>(StateNominal::PX) = x_nominal_predict.segment<3>(StateNominal::PX) + delta_x_.segment<3>(StateError::PX);
  x_nominal_.segment<3>(StateNominal::VX) = x_nominal_predict.segment<3>(StateNominal::VX) + delta_x_.segment<3>(StateError::VX);
  x_nominal_.segment<3>(StateNominal::WX) = x_nominal_predict.segment<3>(StateNominal::WX) + delta_x_.segment<3>(StateError::WX);


  Eigen::Quaterniond orientation_predict;

  orientation_predict.w() = x_nominal_predict(StateNominal::QW);
  orientation_predict.x() = x_nominal_predict(StateNominal::QX);
  orientation_predict.y() = x_nominal_predict(StateNominal::QY);
  orientation_predict.z() = x_nominal_predict(StateNominal::QZ);

  Eigen::Quaterniond orientation_error =
      Eigen::Quaterniond(1, 0.5 * delta_x_(StateError::ROLL), 0.5 * delta_x_(StateError::PITCH), 0.5 * delta_x_(StateError::YAW));


  Eigen::Quaterniond orientation_corrected = (orientation_predict * orientation_error);

  orientation_corrected.normalize();


  x_nominal_(StateNominal::QW) = orientation_corrected.w();
  x_nominal_(StateNominal::QX) = orientation_corrected.x();
  x_nominal_(StateNominal::QY) = orientation_corrected.y();
  x_nominal_(StateNominal::QZ) = orientation_corrected.z();


  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0);
    double          rad2deg = 180.0 / M_PI;

    RCLCPP_DEBUG_STREAM(logger_, "--- [AFTER] - MEKF Correction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[2] * rad2deg << " P:" << euler[1] * rad2deg << " Y:" << euler[0] * rad2deg);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());

    // Opcional: Printar a magnitude do erro aplicado
    RCLCPP_DEBUG_STREAM(logger_, "     └ Delta_x norm: " << delta_x_.norm());
  }

  inject_error_and_reset();
}

void MEKFEstimator::correct(const nav_msgs::msg::Odometry measurements) {

  Eigen::Vector3d    p_meas = Eigen::Vector3d(measurements.pose.pose.position.x, measurements.pose.pose.position.y, measurements.pose.pose.position.z);
  Eigen::Quaterniond q_meas = Eigen::Quaterniond(measurements.pose.pose.orientation.w, measurements.pose.pose.orientation.x,
                                                 measurements.pose.pose.orientation.y, measurements.pose.pose.orientation.z);
  Eigen::Vector3d    v_meas = Eigen::Vector3d(measurements.twist.twist.linear.x, measurements.twist.twist.linear.y, measurements.twist.twist.linear.z);
  Eigen::Vector3d    w_meas = Eigen::Vector3d(measurements.twist.twist.angular.x, measurements.twist.twist.angular.y, measurements.twist.twist.angular.z);

  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler      = q_final.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d euler_mean = q_meas.toRotationMatrix().eulerAngles(0, 1, 2);

    RCLCPP_DEBUG_STREAM(logger_, "--- [BEFORE] - MEKF Correction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[0] << " P:" << euler[1] << " Y:" << euler[2]);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
    RCLCPP_DEBUG_STREAM(logger_, "--- Odometry Measurement ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << measurements.pose.pose.position.x << " " << measurements.pose.pose.position.y << " "
                                                         << measurements.pose.pose.position.z);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << measurements.pose.pose.orientation.w << " " << measurements.pose.pose.orientation.x << " "
                                                           << measurements.pose.pose.orientation.y << " " << measurements.pose.pose.orientation.z);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler_mean[0] << " P:" << euler_mean[1] << " Y:" << euler_mean[2]);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << measurements.twist.twist.linear.x << " " << measurements.twist.twist.linear.y << " "
                                                         << measurements.twist.twist.linear.z);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << measurements.twist.twist.angular.x << " " << measurements.twist.twist.angular.y << " "
                                                         << measurements.twist.twist.angular.z);
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
  }

  Eigen::VectorXd z(13);
  z.segment<3>(StateNominal::PX) = p_meas;
  z.segment<3>(StateNominal::VX) = v_meas;
  z.segment<4>(StateNominal::QW) = Eigen::Vector4d(q_meas.w(), q_meas.x(), q_meas.y(), q_meas.z());
  z.segment<3>(StateNominal::WX) = w_meas;

  Eigen::VectorXd h(13);
  h.segment<3>(StateNominal::PX) = x_nominal_predict.segment<3>(StateNominal::PX);
  h.segment<3>(StateNominal::VX) = x_nominal_predict.segment<3>(StateNominal::VX);
  h.segment<4>(StateNominal::QW) = x_nominal_predict.segment<4>(StateNominal::QW);
  h.segment<3>(StateNominal::WX) = x_nominal_predict.segment<3>(StateNominal::WX);

  Eigen::VectorXd y(12);
  y.segment<3>(StateError::PX) = z.segment<3>(StateNominal::PX) - h.segment<3>(StateNominal::PX);
  y.segment<3>(StateError::VX) = z.segment<3>(StateNominal::VX) - h.segment<3>(StateNominal::VX);
  y.segment<3>(StateError::WX) = z.segment<3>(StateNominal::WX) - h.segment<3>(StateNominal::WX);

  // Calcular o quaternion erro
  Eigen::Quaterniond q_hat(h(StateNominal::QW), h(StateNominal::QX), h(StateNominal::QY), h(StateNominal::QZ));

  Eigen::Quaterniond dq = q_hat.inverse() * q_meas;

  // CRÍTICO: Verificar sinal da componente escalar para evitar ambiguidade
  // Quaternions q e -q representam a mesma rotação, mas escolhemos w >= 0
  if (dq.w() < 0.0) {
    dq.w() = -dq.w();
    dq.x() = -dq.x();
    dq.y() = -dq.y();
    dq.z() = -dq.z();
  }

  // Extrair o vetor de erro (aproximação de pequeno ângulo)
  // dθ ≈ 2 * [x, y, z]^T da parte vetorial do quaternion
  y.segment<3>(StateError::ROLL) = 2.0 * dq.vec();

  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(12, 12);

  Eigen::MatrixXd R                                 = Eigen::MatrixXd::Identity(12, 12);
  R.block<2, 2>(StateError::PX, StateError::PX)     = Eigen::Matrix2d::Identity() * _gains_.odometry.position_xy;
  R(StateError::PZ, StateError::PZ)                 = 1.0 * _gains_.odometry.position_z;
  R.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() * _gains_.odometry.orientation;
  R.block<2, 2>(StateError::VX, StateError::VX)     = Eigen::Matrix2d::Identity() * _gains_.odometry.velocity_linear_xy;
  R(StateError::VZ, StateError::VZ)                 = 1.0 * _gains_.odometry.velocity_linear_z;
  R.block<3, 3>(StateError::WX, StateError::WX)     = Eigen::Matrix3d::Identity() * _gains_.odometry.velocity_angular;

  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  RCLCPP_DEBUG_STREAM(logger_, "Innovation y:");
  RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << y.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << y.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << y.segment<3>(StateError::WX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << y.segment<3>(StateError::ROLL).transpose());

  delta_x_ = K * y;

  RCLCPP_DEBUG_STREAM(logger_, "State Correction delta_x:");
  RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << delta_x_.segment<3>(StateError::WX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << delta_x_.segment<3>(StateError::ROLL).transpose());

  P_                                      = (Eigen::MatrixXd::Identity(12, 12) - K * H) * P_;
  x_nominal_.segment<3>(StateNominal::PX) = x_nominal_predict.segment<3>(StateNominal::PX) + delta_x_.segment<3>(StateError::PX);
  x_nominal_.segment<3>(StateNominal::VX) = x_nominal_predict.segment<3>(StateNominal::VX) + delta_x_.segment<3>(StateError::VX);
  x_nominal_.segment<3>(StateNominal::WX) = x_nominal_predict.segment<3>(StateNominal::WX) + delta_x_.segment<3>(StateError::WX);


  Eigen::Quaterniond orientation_predict;

  orientation_predict.w() = x_nominal_predict(StateNominal::QW);
  orientation_predict.x() = x_nominal_predict(StateNominal::QX);
  orientation_predict.y() = x_nominal_predict(StateNominal::QY);
  orientation_predict.z() = x_nominal_predict(StateNominal::QZ);

  Eigen::Quaterniond orientation_error =
      Eigen::Quaterniond(1, 0.5 * delta_x_(StateError::ROLL), 0.5 * delta_x_(StateError::PITCH), 0.5 * delta_x_(StateError::YAW));


  Eigen::Quaterniond orientation_corrected = (orientation_predict * orientation_error);

  orientation_corrected.normalize();


  x_nominal_(StateNominal::QW) = orientation_corrected.w();
  x_nominal_(StateNominal::QX) = orientation_corrected.x();
  x_nominal_(StateNominal::QY) = orientation_corrected.y();
  x_nominal_(StateNominal::QZ) = orientation_corrected.z();


  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0);
    double          rad2deg = 180.0 / M_PI;

    RCLCPP_DEBUG_STREAM(logger_, "--- [AFTER] - MEKF Correction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[2] * rad2deg << " P:" << euler[1] * rad2deg << " Y:" << euler[0] * rad2deg);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());

    // Opcional: Printar a magnitude do erro aplicado
    RCLCPP_DEBUG_STREAM(logger_, "     └ Delta_x norm: " << delta_x_.norm());
  }

  inject_error_and_reset();
}


void MEKFEstimator::inject_error_and_reset() {
  Eigen::MatrixXd G                                 = Eigen::MatrixXd::Identity(12, 12);
  G.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() - 0.5 * skew_symmetric(delta_x_.segment<3>(StateError::ROLL));
  P_                                                = G * P_ * G.transpose();
  delta_x_.setZero();
}


Eigen::Matrix3d MEKFEstimator::skew_symmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew;
  skew << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return skew;
}

Eigen::Quaterniond MEKFEstimator::ExpSO3Quaternion(const Eigen::Vector3d &theta_vec) {
  Eigen::Quaterniond delta_q;
  if (theta_vec.norm() < 1e-12) {
    delta_q = Eigen::Quaterniond(1.0, 0.5 * theta_vec.x(), 0.5 * theta_vec.y(), 0.5 * theta_vec.z());
  } else {
    delta_q.w()   = (cos(0.5 * theta_vec.norm()));
    delta_q.vec() = theta_vec / theta_vec.norm() * sin(0.5 * theta_vec.norm());
  }
  return delta_q.normalized();
}

Eigen::Vector3d MEKFEstimator::get_position() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::PX), x_nominal_(StateNominal::PY), x_nominal_(StateNominal::PZ));
}

Eigen::Quaterniond MEKFEstimator::get_orientation() const {
  return Eigen::Quaterniond(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
}

Eigen::Vector3d MEKFEstimator::get_linear_velocity() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::VX), x_nominal_(StateNominal::VY), x_nominal_(StateNominal::VZ));
}

Eigen::Vector3d MEKFEstimator::get_angular_velocity() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::WX), x_nominal_(StateNominal::WY), x_nominal_(StateNominal::WZ));
}

nav_msgs::msg::Odometry MEKFEstimator::get_odometry() const {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x    = x_nominal_(StateNominal::PX);
  odom.pose.pose.position.y    = x_nominal_(StateNominal::PY);
  odom.pose.pose.position.z    = x_nominal_(StateNominal::PZ);
  odom.pose.pose.orientation.w = x_nominal_(StateNominal::QW);
  odom.pose.pose.orientation.x = x_nominal_(StateNominal::QX);
  odom.pose.pose.orientation.y = x_nominal_(StateNominal::QY);
  odom.pose.pose.orientation.z = x_nominal_(StateNominal::QZ);
  odom.twist.twist.linear.x    = x_nominal_(StateNominal::VX);
  odom.twist.twist.linear.y    = x_nominal_(StateNominal::VY);
  odom.twist.twist.linear.z    = x_nominal_(StateNominal::VZ);
  odom.twist.twist.angular.x   = x_nominal_(StateNominal::WX);
  odom.twist.twist.angular.y   = x_nominal_(StateNominal::WY);
  odom.twist.twist.angular.z   = x_nominal_(StateNominal::WZ);
  return odom;
}

Eigen::MatrixXd MEKFEstimator::get_covariance() const {
  return P_;
}

/* set_measurement_noise_gains() //{ */
void MEKFEstimator::set_measurement_noise_gains(const MeasurementNoiseGains &gains) {
  _gains_ = gains;
}
//}

/* set_verbosity() //{ */
void MEKFEstimator::set_verbosity(const std::string &verbosity) {
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
}
//}

}  // namespace laser_uav_estimators
