#include <laser_uav_estimators/mekf_state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace laser_uav_estimators
{

/* MEKFEstimator() //{ */
MEKFEstimator::MEKFEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia,
                             const MeasurementNoiseGains &measurement_noise_px4, const MeasurementNoiseGains &measurement_noise_fast_lio,
                             const MeasurementNoiseGains &measurement_noise_openvins, const MeasurementNoiseGains &measurement_noise_garmin,
                             const NoiseGains &process_noise, const std::string &verbosity)
    : _measurement_noise_px4_(measurement_noise_px4),
      _measurement_noise_fast_lio_(measurement_noise_fast_lio),
      _measurement_noise_openvins_(measurement_noise_openvins),
      _measurement_noise_garmin_(measurement_noise_garmin),
      _process_noise_(process_noise),
      _mass_(mass),
      _allocation_matrix_(allocation_matrix),
      _inertia_(inertia),
      logger_(rclcpp::get_logger("multi_mekf_state_estimator")) {
  set_verbosity(verbosity);
  std::cout << "Verbosity: " << verbosity << std::endl;
  if (verbosity == "DEBUG") {
    is_debug_ = true;
  }
  RCLCPP_INFO(logger_, "--- MULTI MEKF STATE ESTIMATOR CONSTRUCTOR ---");

  x_nominal_                   = Eigen::VectorXd::Zero(13);  // Posição (3), Orientação (4), Velocidade Linear (3), Velocidade Angular (3)
  x_nominal_(StateNominal::QW) = 1.0;                        // Inicializa a orientação como identidade


  delta_x_ = Eigen::VectorXd::Zero(12);                // Erros em Posição (3), Orientação (3), Velocidade Linear (3),Velocidade Angular (3)
  P_       = Eigen::MatrixXd::Identity(12, 12) * 0.1;  // Covariância inicial pequena
}
//}

/* predict() //{ */
void MEKFEstimator::predict(const Eigen::VectorXd &u, double dt) {
  /* if (is_debug_) { */
  /*   // Converter a orientação corrigida para Euler para facilitar a leitura */
  /*   Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ)); */

  /*   // Ordem Z-Y-X -> Yaw, Pitch, Roll */
  /*   Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0); */
  /*   double          rad2deg = 180.0 / M_PI; */

  /*   RCLCPP_DEBUG_STREAM(logger_, "--- [BEFORE] - MULTI MEKF Prediction Update ---"); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  " */
  /*                                    << "R:" << euler[2] * rad2deg << " P:" << euler[1] * rad2deg << " Y:" << euler[0] * rad2deg); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------"); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "--- - Entrada:  ---"); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ u:          " << u.transpose()); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "     ├ dt:         " << dt); */
  /*   RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------"); */
  /* } */

  Eigen::Vector3d    position = x_nominal_.segment<3>(StateNominal::PX);
  Eigen::Quaterniond orientation =
      Eigen::Quaterniond(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
  Eigen::Vector3d linear_velocity  = x_nominal_.segment<3>(StateNominal::VX);
  Eigen::Vector3d angular_velocity = x_nominal_.segment<3>(StateNominal::WX);

  Eigen::Matrix3d R_body_to_inertial = orientation.toRotationMatrix();

  Eigen::Matrix<double, 4, 1> force_body = _allocation_matrix_ * u;
  double                      thrust     = force_body(0);
  Eigen::Vector3d             tau        = force_body.segment<3>(1);
  Eigen::Vector3d             bz         = R_body_to_inertial * Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d             ez         = Eigen::Vector3d(0.0, 0.0, 1.0);

  Eigen::Vector3d position_predict =
      position + dt * (R_body_to_inertial * linear_velocity) + ((0.5 * dt * dt) * (R_body_to_inertial * (thrust / _mass_) * bz + (GRAVITY * ez)));
  Eigen::Vector3d linear_velocity_predict =
      linear_velocity + dt * ((thrust / _mass_) * bz + R_body_to_inertial.transpose() * GRAVITY * ez - angular_velocity.cross(linear_velocity));

  Eigen::Vector3d angular_velocity_predict = angular_velocity + dt * (_inertia_.inverse() * (-angular_velocity.cross((_inertia_ * angular_velocity)) + tau));

  Eigen::Vector3d    half_theta_vec      = (dt / 2) * angular_velocity;
  Eigen::Quaterniond delta_q             = ExpSO3Quaternion(half_theta_vec);
  Eigen::Quaterniond orientation_predict = orientation * delta_q;
  orientation_predict.normalize();

  x_nominal_predict                   = Eigen::VectorXd::Zero(13);
  x_nominal_predict(StateNominal::QW) = 1.0;

  x_nominal_predict.segment<3>(StateNominal::PX) = position_predict;
  x_nominal_predict.segment<4>(StateNominal::QW) =
      Eigen::Vector4d(orientation_predict.w(), orientation_predict.x(), orientation_predict.y(), orientation_predict.z());
  x_nominal_predict.segment<3>(StateNominal::VX) = linear_velocity_predict;
  x_nominal_predict.segment<3>(StateNominal::WX) = angular_velocity_predict;


  Eigen::Matrix3d Sv   = skew_symmetric(linear_velocity);
  Eigen::Matrix3d Sw   = skew_symmetric(angular_velocity);
  Eigen::Matrix3d SwIq = skew_symmetric(_inertia_ * angular_velocity);
  Eigen::Matrix3d Sa   = skew_symmetric((thrust / _mass_) * bz);
  Eigen::Matrix3d SRg  = skew_symmetric(R_body_to_inertial.transpose() * GRAVITY * ez);

  Eigen::Matrix3d fp_theta = -dt * R_body_to_inertial * Sv - 0.5 * dt * dt * R_body_to_inertial * Sa;
  Eigen::Matrix3d fww      = Eigen::Matrix3d::Identity() - dt * (_inertia_.inverse() * (Sw * _inertia_ - SwIq));

  Eigen::MatrixXd Fx                                 = Eigen::MatrixXd::Zero(12, 12);
  Fx.block<3, 3>(StateError::PX, StateError::PX)     = Eigen::Matrix3d::Identity();
  Fx.block<3, 3>(StateError::PX, StateError::ROLL)   = fp_theta;
  Fx.block<3, 3>(StateError::PX, StateError::VX)     = dt * R_body_to_inertial;
  Fx.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() - dt * Sw;
  Fx.block<3, 3>(StateError::ROLL, StateError::WX)   = dt * Eigen::Matrix3d::Identity();
  Fx.block<3, 3>(StateError::VX, StateError::ROLL)   = -dt * SRg;
  Fx.block<3, 3>(StateError::VX, StateError::VX)     = Eigen::Matrix3d::Identity() - dt * Sw;
  Fx.block<3, 3>(StateError::VX, StateError::WX)     = -dt * Sv;
  Fx.block<3, 3>(StateError::WX, StateError::WX)     = fww;

  Eigen::MatrixXd Q                       = Eigen::MatrixXd::Identity(12, 12);
  Q(StateError::PX, StateError::PX)       = 1.0 * _process_noise_.position.x;
  Q(StateError::PY, StateError::PY)       = 1.0 * _process_noise_.position.y;
  Q(StateError::PZ, StateError::PZ)       = 1.0 * _process_noise_.position.z;
  Q(StateError::ROLL, StateError::ROLL)   = 1.0 * _process_noise_.orientation.roll;
  Q(StateError::PITCH, StateError::PITCH) = 1.0 * _process_noise_.orientation.pitch;
  Q(StateError::YAW, StateError::YAW)     = 1.0 * _process_noise_.orientation.yaw;
  Q(StateError::VX, StateError::VX)       = 1.0 * _process_noise_.linear_velocity.vx;
  Q(StateError::VY, StateError::VY)       = 1.0 * _process_noise_.linear_velocity.vy;
  Q(StateError::VZ, StateError::VZ)       = 1.0 * _process_noise_.linear_velocity.vz;
  Q(StateError::WX, StateError::WX)       = 1.0 * _process_noise_.angular_velocity.wx;
  Q(StateError::WY, StateError::WY)       = 1.0 * _process_noise_.angular_velocity.wy;
  Q(StateError::WZ, StateError::WZ)       = 1.0 * _process_noise_.angular_velocity.wz;

  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_predict(StateNominal::QW), x_nominal_predict(StateNominal::QX), x_nominal_predict(StateNominal::QY),
                               x_nominal_predict(StateNominal::QZ));

    // Ordem Z-Y-X -> Yaw, Pitch, Roll
    Eigen::Vector3d euler   = q_final.toRotationMatrix().eulerAngles(2, 1, 0);
    double          rad2deg = 180.0 / M_PI;

    RCLCPP_DEBUG_STREAM(logger_, "--- MULTI MEKF Prediction Update ---");
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
//}

/* correct() //{ */
void MEKFEstimator::correct(const MeasurementPackage &measurements) {
  int                                total_sources = 0;
  std::vector<std::string>           meas_type;
  std::vector<Eigen::Vector3d>       p_meas;
  std::vector<Eigen::Quaterniond>    q_meas;
  std::vector<Eigen::Vector3d>       v_meas;
  std::vector<Eigen::Vector3d>       w_meas;
  std::vector<MeasurementNoiseGains> noise_meas;
  if (measurements.px4) {
    p_meas.push_back(Eigen::Vector3d(measurements.px4.value().pose.pose.position.x, measurements.px4.value().pose.pose.position.y,
                                     measurements.px4.value().pose.pose.position.z));
    q_meas.push_back(Eigen::Quaterniond(measurements.px4.value().pose.pose.orientation.w, measurements.px4.value().pose.pose.orientation.x,
                                        measurements.px4.value().pose.pose.orientation.y, measurements.px4.value().pose.pose.orientation.z));
    v_meas.push_back(Eigen::Vector3d(measurements.px4.value().twist.twist.linear.x, measurements.px4.value().twist.twist.linear.y,
                                     measurements.px4.value().twist.twist.linear.z));
    w_meas.push_back(Eigen::Vector3d(measurements.px4.value().twist.twist.angular.x, measurements.px4.value().twist.twist.angular.y,
                                     measurements.px4.value().twist.twist.angular.z));
    noise_meas.push_back(_measurement_noise_px4_);
    meas_type.push_back("px4");
    total_sources += 1;
  }
  if (measurements.fast_lio) {
    p_meas.push_back(Eigen::Vector3d(measurements.fast_lio.value().pose.pose.position.x, measurements.fast_lio.value().pose.pose.position.y,
                                     measurements.fast_lio.value().pose.pose.position.z));
    q_meas.push_back(Eigen::Quaterniond(measurements.fast_lio.value().pose.pose.orientation.w, measurements.fast_lio.value().pose.pose.orientation.x,
                                        measurements.fast_lio.value().pose.pose.orientation.y, measurements.fast_lio.value().pose.pose.orientation.z));
    v_meas.push_back(Eigen::Vector3d(measurements.fast_lio.value().twist.twist.linear.x, measurements.fast_lio.value().twist.twist.linear.y,
                                     measurements.fast_lio.value().twist.twist.linear.z));
    w_meas.push_back(Eigen::Vector3d(measurements.fast_lio.value().twist.twist.angular.x, measurements.fast_lio.value().twist.twist.angular.y,
                                     measurements.fast_lio.value().twist.twist.angular.z));
    noise_meas.push_back(_measurement_noise_fast_lio_);
    meas_type.push_back("fast-lio");
    total_sources += 1;
  }
  if (measurements.openvins) {
    p_meas.push_back(Eigen::Vector3d(measurements.openvins.value().pose.pose.position.x, measurements.openvins.value().pose.pose.position.y,
                                     measurements.openvins.value().pose.pose.position.z));
    q_meas.push_back(Eigen::Quaterniond(measurements.openvins.value().pose.pose.orientation.w, measurements.openvins.value().pose.pose.orientation.x,
                                        measurements.openvins.value().pose.pose.orientation.y, measurements.openvins.value().pose.pose.orientation.z));
    v_meas.push_back(Eigen::Vector3d(measurements.openvins.value().twist.twist.linear.x, measurements.openvins.value().twist.twist.linear.y,
                                     measurements.openvins.value().twist.twist.linear.z));
    w_meas.push_back(Eigen::Vector3d(measurements.openvins.value().twist.twist.angular.x, measurements.openvins.value().twist.twist.angular.y,
                                     measurements.openvins.value().twist.twist.angular.z));
    noise_meas.push_back(_measurement_noise_openvins_);
    meas_type.push_back("openvins");
    total_sources += 1;
  }
  if (measurements.garmin) {
    p_meas.push_back(Eigen::Vector3d(0, 0, measurements.garmin.value().range));
    q_meas.push_back(Eigen::Quaterniond(1, 0, 0, 0));
    v_meas.push_back(Eigen::Vector3d(0, 0, 0));
    w_meas.push_back(Eigen::Vector3d(0, 0, 0));
    noise_meas.push_back(_measurement_noise_garmin_);
    meas_type.push_back("garmin");
    total_sources += 1;
  }

  if (total_sources == 0) {
    RCLCPP_DEBUG_STREAM(logger_, "No measurements available. Skipping correction step.");
    return;
  }
  if (is_debug_) {
    // Converter a orientação corrigida para Euler para facilitar a leitura
    Eigen::Quaterniond q_final(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));

    Eigen::Vector3d euler = q_final.toRotationMatrix().eulerAngles(0, 1, 2);
    RCLCPP_DEBUG_STREAM(logger_, "--- MULTI MEKF Correction Update ---");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_nominal_.segment<3>(StateNominal::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << x_nominal_.segment<4>(StateNominal::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                     << "R:" << euler[0] << " P:" << euler[1] << " Y:" << euler[2]);
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_nominal_.segment<3>(StateNominal::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << x_nominal_.segment<3>(StateNominal::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Trace(P):     " << P_.trace());
    RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");

    for (auto i = 0; i < total_sources; i++) {
      Eigen::Vector3d euler_mean = q_meas[i].toRotationMatrix().eulerAngles(0, 1, 2);
      RCLCPP_DEBUG_STREAM(logger_, "--- Odometry Measurement " << meas_type[i] << " ---");
      RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << p_meas[i].x() << " " << p_meas[i].y() << " " << p_meas[i].z());
      RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat (w,x,y,z): " << q_meas[i].w() << " " << q_meas[i].x() << " " << q_meas[i].y() << " " << q_meas[i].z());
      RCLCPP_DEBUG_STREAM(logger_, "     ├ Euler (deg):  "
                                       << "R:" << euler_mean[0] << " P:" << euler_mean[1] << " Y:" << euler_mean[2]);
      RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << v_meas[i].x() << " " << v_meas[i].y() << " " << v_meas[i].z());
      RCLCPP_DEBUG_STREAM(logger_, "     ├ Ang. Vel.:    " << w_meas[i].x() << " " << w_meas[i].y() << " " << w_meas[i].z());
      RCLCPP_DEBUG_STREAM(logger_, "------------------------------------------");
    }
  }

  Eigen::VectorXd z = Eigen::VectorXd::Zero(total_sources * 13);
  for (auto i = 0; i < total_sources; i++) {
    z.segment<3>(StateNominal::PX + i * 13) = p_meas[i];
    z.segment<4>(StateNominal::QW + i * 13) = Eigen::Vector4d(q_meas[i].w(), q_meas[i].x(), q_meas[i].y(), q_meas[i].z());
    z.segment<3>(StateNominal::VX + i * 13) = v_meas[i];
    z.segment<3>(StateNominal::WX + i * 13) = w_meas[i];
  }

  Eigen::VectorXd h(13);
  h.segment<3>(StateNominal::PX) = x_nominal_predict.segment<3>(StateNominal::PX);
  h.segment<4>(StateNominal::QW) = x_nominal_predict.segment<4>(StateNominal::QW);
  h.segment<3>(StateNominal::VX) = x_nominal_predict.segment<3>(StateNominal::VX);
  h.segment<3>(StateNominal::WX) = x_nominal_predict.segment<3>(StateNominal::WX);

  Eigen::VectorXd y(total_sources * 12);
  for (auto i = 0; i < total_sources; i++) {
    y.segment<3>(StateError::PX + i * 12) = z.segment<3>(StateNominal::PX + i * 13) - h.segment<3>(StateNominal::PX);
    y.segment<3>(StateError::ROLL + i * 12) =
        2.0 * (Eigen::Quaterniond(h(StateNominal::QW), h(StateNominal::QX), h(StateNominal::QY), h(StateNominal::QZ)).inverse() *
               Eigen::Quaterniond(z(StateNominal::QW + i * 13), z(StateNominal::QX + i * 13), z(StateNominal::QY + i * 13), z(StateNominal::QZ + i * 13)))
                  .vec();
    y.segment<3>(StateError::VX + i * 12) = z.segment<3>(StateNominal::VX + i * 13) - h.segment<3>(StateNominal::VX);
    y.segment<3>(StateError::WX + i * 12) = z.segment<3>(StateNominal::WX + i * 13) - h.segment<3>(StateNominal::WX);
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_sources * 12, 12);
  for (auto i = 0; i < total_sources; i++) {
    if (meas_type[i] != "garmin") {
      H.block(i * 12, 0, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    } else {
      H.block(i * 12, 0, 12, 12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0;
    }
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(total_sources * 12, total_sources * 12);
  for (auto i = 0; i < total_sources; i++) {
    R(StateError::PX + i * 12, StateError::PX + i * 12)       = 1.0 * noise_meas[i].odometry.position.x;
    R(StateError::PY + i * 12, StateError::PY + i * 12)       = 1.0 * noise_meas[i].odometry.position.y;
    R(StateError::PZ + i * 12, StateError::PZ + i * 12)       = 1.0 * noise_meas[i].odometry.position.z;
    R(StateError::ROLL + i * 12, StateError::ROLL + i * 12)   = 1.0 * noise_meas[i].odometry.orientation.roll;
    R(StateError::PITCH + i * 12, StateError::PITCH + i * 12) = 1.0 * noise_meas[i].odometry.orientation.pitch;
    R(StateError::YAW + i * 12, StateError::YAW + i * 12)     = 1.0 * noise_meas[i].odometry.orientation.yaw;
    R(StateError::VX + i * 12, StateError::VX + i * 12)       = 1.0 * noise_meas[i].odometry.linear_velocity.vx;
    R(StateError::VY + i * 12, StateError::VY + i * 12)       = 1.0 * noise_meas[i].odometry.linear_velocity.vy;
    R(StateError::VZ + i * 12, StateError::VZ + i * 12)       = 1.0 * noise_meas[i].odometry.linear_velocity.vz;
    R(StateError::WX + i * 12, StateError::WX + i * 12)       = 1.0 * noise_meas[i].odometry.angular_velocity.wx;
    R(StateError::WY + i * 12, StateError::WY + i * 12)       = 1.0 * noise_meas[i].odometry.angular_velocity.wy;
    R(StateError::WZ + i * 12, StateError::WZ + i * 12)       = 1.0 * noise_meas[i].odometry.angular_velocity.wz;
  }

  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  for (auto i = 0; i < total_sources; i++) {
    RCLCPP_DEBUG_STREAM(logger_, "Innovation y:");
    RCLCPP_DEBUG_STREAM(logger_, "     ├  pos. " << meas_type[i] << ": " << y.segment<3>(StateError::PX + i * 12).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └  ori. " << meas_type[i] << ": " << y.segment<3>(StateError::ROLL + i * 12).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  vel. " << meas_type[i] << ": " << y.segment<3>(StateError::VX + i * 12).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├  ang. " << meas_type[i] << ": " << y.segment<3>(StateError::WX + i * 12).transpose());
  }
  delta_x_ = K * y;

  RCLCPP_DEBUG_STREAM(logger_, "State Correction delta_x:");
  RCLCPP_DEBUG_STREAM(logger_, "     ├  pos.:   " << delta_x_.segment<3>(StateError::PX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     └  ori.:   " << delta_x_.segment<3>(StateError::ROLL).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  vel.:   " << delta_x_.segment<3>(StateError::VX).transpose());
  RCLCPP_DEBUG_STREAM(logger_, "     ├  ang.:   " << delta_x_.segment<3>(StateError::WX).transpose());

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
  orientation_error.normalize();

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

    RCLCPP_DEBUG_STREAM(logger_, "--- [AFTER] - MULTI MEKF Correction Update ---");
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
//}

/* inject_error_and_reset() //{ */
void MEKFEstimator::inject_error_and_reset() {
  Eigen::MatrixXd G                                 = Eigen::MatrixXd::Identity(12, 12);
  G.block<3, 3>(StateError::ROLL, StateError::ROLL) = Eigen::Matrix3d::Identity() - 0.5 * skew_symmetric(delta_x_.segment<3>(StateError::ROLL));
  P_                                                = G * P_ * G.transpose();
  delta_x_.setZero();
}
//}

/* skew_symmetric() //{ */
Eigen::Matrix3d MEKFEstimator::skew_symmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew;
  skew << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return skew;
}
//}

/* ExpSO3Quaternion //{ */
Eigen::Quaterniond MEKFEstimator::ExpSO3Quaternion(const Eigen::Vector3d &theta_vec) {
  Eigen::Quaterniond delta_q;
  if (theta_vec.norm() < 1e-8) {
    delta_q = Eigen::Quaterniond(1.0, 0.5 * theta_vec.x(), 0.5 * theta_vec.y(), 0.5 * theta_vec.z());
  } else {
    delta_q.w()   = (cos(0.5 * theta_vec.norm()));
    delta_q.vec() = theta_vec / theta_vec.norm() * sin(0.5 * theta_vec.norm());
  }
  return delta_q.normalized();
}
//}

/* get_position() //{ */
Eigen::Vector3d MEKFEstimator::get_position() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::PX), x_nominal_(StateNominal::PY), x_nominal_(StateNominal::PZ));
}
//}

/* get_orientation() //{ */
Eigen::Quaterniond MEKFEstimator::get_orientation() const {
  return Eigen::Quaterniond(x_nominal_(StateNominal::QW), x_nominal_(StateNominal::QX), x_nominal_(StateNominal::QY), x_nominal_(StateNominal::QZ));
}
//}

/* get_linear_velocity() //{ */
Eigen::Vector3d MEKFEstimator::get_linear_velocity() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::VX), x_nominal_(StateNominal::VY), x_nominal_(StateNominal::VZ));
}
//}

/* get_angular_velocity() //{ */
Eigen::Vector3d MEKFEstimator::get_angular_velocity() const {
  return Eigen::Vector3d(x_nominal_(StateNominal::WX), x_nominal_(StateNominal::WY), x_nominal_(StateNominal::WZ));
}
//}

/* get_odometry() //{ */
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
//}

/* get_covariance() //{ */
Eigen::MatrixXd MEKFEstimator::get_covariance() const {
  return P_;
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