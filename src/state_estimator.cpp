#include <laser_uav_estimators/state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace laser_uav_estimators
{
/* StateEstimator() //{ */
StateEstimator::StateEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const std::string &verbosity,
                               const std::vector<double> &irr_position_a, const std::vector<double> &irr_position_b, const std::vector<double> &irr_velocity_a,
                               const std::vector<double> &irr_velocity_b, const std::vector<double> &irr_angular_velocity_a,
                               const std::vector<double> &irr_angular_velocity_b)
    : mass_(mass),
      allocation_matrix_(allocation_matrix),
      n_inputs_(allocation_matrix.cols()),
      inertia_tensor_(inertia),
      logger_(rclcpp::get_logger("StateEstimator")),
      q_gains_(),
      r_gains_() {
  set_verbosity(verbosity);
  RCLCPP_INFO(logger_, "--- STATE ESTIMATOR CONSTRUCTOR ---");

  inertia_tensor_inv_ = inertia.inverse();

  x_.setZero();
  x_(State::QW) = 1.0;

  P_.setIdentity();
  P_ *= 0.1;

  F_.setIdentity();
  Q_.setIdentity();
  update_Q_matrix();

  imu_propagator_ = laser_uav_lib::ImuPropagator();

  std::cout << "Initializing IIR Filters..." << std::endl;
  std::cout << "  Position IIR a coeffs: ";
  for (const auto &a_coeff : irr_position_a)
    std::cout << a_coeff << " ";
  std::cout << std::endl;
  std::cout << "  Position IIR b coeffs: ";
  for (const auto &b_coeff : irr_position_b)
    std::cout << b_coeff << " ";
  std::cout << std::endl;
  std::cout << "  Velocity IIR a coeffs: ";
  for (const auto &a_coeff : irr_velocity_a)
    std::cout << a_coeff << " ";
  std::cout << std::endl;
  std::cout << "  Velocity IIR b coeffs: ";
  for (const auto &b_coeff : irr_velocity_b)
    std::cout << b_coeff << " ";
  std::cout << std::endl;
  std::cout << "  Angular Velocity IIR a coeffs: ";
  for (const auto &a_coeff : irr_angular_velocity_a)
    std::cout << a_coeff << " ";
  std::cout << std::endl;
  std::cout << "  Angular Velocity IIR b coeffs: ";
  for (const auto &b_coeff : irr_angular_velocity_b)
    std::cout << b_coeff << " ";
  std::cout << std::endl;

  pos_x_filter_ = laser_uav_lib::IIRFilter(irr_position_a, irr_position_b);
  pos_y_filter_ = laser_uav_lib::IIRFilter(irr_position_a, irr_position_b);
  pos_z_filter_ = laser_uav_lib::IIRFilter(irr_position_a, irr_position_b);

  vel_x_filter_ = laser_uav_lib::IIRFilter(irr_velocity_a, irr_velocity_b);
  vel_y_filter_ = laser_uav_lib::IIRFilter(irr_velocity_a, irr_velocity_b);
  vel_z_filter_ = laser_uav_lib::IIRFilter(irr_velocity_a, irr_velocity_b);

  ang_vel_x_filter_ = laser_uav_lib::IIRFilter(irr_angular_velocity_a, irr_angular_velocity_b);
  ang_vel_y_filter_ = laser_uav_lib::IIRFilter(irr_angular_velocity_a, irr_angular_velocity_b);
  ang_vel_z_filter_ = laser_uav_lib::IIRFilter(irr_angular_velocity_a, irr_angular_velocity_b);

  RCLCPP_INFO(logger_, "IIR Filters initialized.");

  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "Output: Initial state x_ = ");
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_.template segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat:         " << x_.template segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_.template segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "     └ Ang. Vel.:    " << x_.template segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "Output: Initial uncertainty (trace P) = " << P_.trace());
  }
}
//}

/* set_process_noise_gains() //{ */
void StateEstimator::set_process_noise_gains(const ProcessNoiseGains &gains) {
  RCLCPP_INFO(logger_, "Updating process noise gains (Q).");
  q_gains_ = gains;
  update_Q_matrix();
}
//}

/* set_measurement_noise_gains() //{ */
void StateEstimator::set_measurement_noise_gains(const MeasurementNoiseGains &gains) {
  RCLCPP_INFO(logger_, "Updating measurement noise gains (R).");
  r_gains_ = gains;
}
//}

/* update_Q_matrix() //{ */
void StateEstimator::update_Q_matrix() {
  Q_.setZero();
  Q_.block<3, 3>(State::PX, State::PX) = Eigen::Matrix3d::Identity() * q_gains_.position;
  Q_.block<4, 4>(State::QW, State::QW) = Eigen::Matrix4d::Identity() * q_gains_.orientation;
  Q_.block<3, 3>(State::VX, State::VX) = Eigen::Matrix3d::Identity() * q_gains_.linear_velocity;
  Q_.block<3, 3>(State::WX, State::WX) = Eigen::Matrix3d::Identity() * q_gains_.angular_velocity;

  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "Process noise matrix Q updated. Trace: " << Q_.trace());
  }
}
//}

/* reset() //{ */
void StateEstimator::reset() {
  RCLCPP_INFO(logger_, "--- EKF FILTER RESET ---");

  x_.setZero();
  x_(State::QW) = 1.0;

  P_.setIdentity();
  P_ *= 0.1;

  F_.setIdentity();
  update_Q_matrix();

  RCLCPP_INFO(logger_, "State and uncertainty reset to default values.");
}
//}

/* set_verbosity() //{ */
void StateEstimator::set_verbosity(const std::string &verbosity) {
  verbosity_ = verbosity;
  is_debug_  = (verbosity_ == "DEBUG" || verbosity_ == "ALL");

  if (verbosity_ == "SILENT") {
    logger_.set_level(rclcpp::Logger::Level::Fatal);
  } else if (verbosity_ == "ERROR") {
    logger_.set_level(rclcpp::Logger::Level::Error);
  } else if (verbosity_ == "WARNING") {
    logger_.set_level(rclcpp::Logger::Level::Warn);
  } else if (is_debug_) {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  } else {
    logger_.set_level(rclcpp::Logger::Level::Info);
  }

  RCLCPP_INFO_STREAM(logger_, "Verbosity level set to: " << verbosity_);
}
//}

/* get_verbosity() //{ */
std::string StateEstimator::get_verbosity() const {
  return verbosity_;
}
//}

/* state_transition_model() //{ */
template <typename T>
Eigen::Matrix<T, STATES, 1> StateEstimator::state_transition_model(const Eigen::Matrix<T, STATES, 1> &x, const Eigen::Matrix<T, Eigen::Dynamic, 1> &u) const {
  Eigen::Quaternion<T> q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
  q.normalize();
  Eigen::Matrix<T, 3, 1>      v_body = x.template segment<3>(State::VX);
  Eigen::Matrix<T, 3, 1>      w_body = x.template segment<3>(State::WX);
  Eigen::Matrix<T, STATES, 1> x_dot;

  x_dot.template segment<3>(State::PX) = q.toRotationMatrix() * v_body;

  Eigen::Quaternion<T> w_quat(T(0), w_body.x(), w_body.y(), w_body.z());
  Eigen::Quaternion<T> q_dot_quat      = w_quat * q.inverse();
  x_dot.template segment<4>(State::QW) = T(0.5) * Eigen::Matrix<T, 4, 1>(q_dot_quat.w(), q_dot_quat.x(), q_dot_quat.y(), q_dot_quat.z());

  Eigen::Matrix<T, 4, 1> wrench = allocation_matrix_.template cast<T>() * u;

  T                      total_thrust = wrench(0);
  Eigen::Matrix<T, 3, 1> thrust_force_body(T(0), T(0), total_thrust);

  Eigen::Matrix<T, 3, 1> tau = wrench.template segment<3>(1);

  Eigen::Matrix<T, 3, 1> g_inertial(T(0), T(0), T(-GRAVITY));
  Eigen::Matrix<T, 3, 1> g_body = q.toRotationMatrix().transpose() * g_inertial;

  x_dot.template segment<3>(State::VX) = (thrust_force_body / T(mass_)) + g_body - w_body.cross(v_body);

  x_dot.template segment<3>(State::WX) = inertia_tensor_inv_.template cast<T>() * (tau - w_body.cross(inertia_tensor_.template cast<T>() * w_body));

  if (std::is_same<T, double>::value && is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "    [state_transition_model] Análise de Aceleração Angular:");
    RCLCPP_DEBUG_STREAM(logger_, "      ├─ w_body (Vel. Atual):  " << w_body.transpose());
    RCLCPP_DEBUG_STREAM(logger_, "      ├─ tau (Torque Motor):   " << tau.transpose());
    RCLCPP_DEBUG_STREAM(logger_, "      └─ x_dot_w (Aceleração): " << x_dot.template segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "      └─ wrench: \n" << wrench);
    RCLCPP_DEBUG_STREAM(logger_, "      └─ allocation_matrix_: \n" << allocation_matrix_);
    RCLCPP_DEBUG_STREAM(logger_, "      └─ inertia_tensor_: \n" << inertia_tensor_);
  }

  return x_dot;
}
//}

/* calculate_jacobian_F() //{ */
void StateEstimator::calculate_jacobian_F(const Eigen::Matrix<double, STATES, 1> &x, const Eigen::VectorXd &u) {
  autodiff::VectorXreal x_ad = x;
  autodiff::VectorXreal u_ad = u;

  auto model_for_autodiff = [&](const autodiff::VectorXreal &x_arg) -> autodiff::VectorXreal {
    return x_arg + this->state_transition_model<autodiff::real>(x_arg, u_ad.cast<autodiff::real>()) * this->dt_;
  };

  F_ = autodiff::jacobian(model_for_autodiff, wrt(x_ad), at(x_ad));
}
//}

/* predict() //{ */
void StateEstimator::predict(const Eigen::VectorXd &u, double dt) {
  RCLCPP_DEBUG_STREAM(logger_, "--- PREDICT ---");
  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "Inputs: u = " << u.transpose() << ", dt = " << dt);
    RCLCPP_DEBUG_STREAM(logger_, "State (x) BEFORE:");
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) BEFORE: " << P_.trace());
  }

  dt_ = dt;

  calculate_jacobian_F(x_, u);

  P_ = F_ * P_ * F_.transpose() + Q_;

  Eigen::Matrix<double, STATES, 1> x_dot = state_transition_model<double>(x_, u);
  x_ += x_dot * dt;

  x_.segment<4>(State::QW).normalize();

  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "State (x_dot):");
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_dot.segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_dot.segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_dot.segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_dot.segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "State (x) AFTER:");
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) AFTER: " << P_.trace());
  }
}
//}

/* correct() //{ */
void StateEstimator::correct(const MeasurementPackage &measurements) {
  RCLCPP_DEBUG_STREAM(logger_, "--- CORRECT ---");

  int           total_measurements = 0;
  constexpr int ODOM_MEASUREMENTS  = 13;
  constexpr int IMU_MEASUREMENTS   = 10;
  constexpr int GPS_MEASUREMENTS   = 2;

  bool has_q_measurement           = false;
  bool should_reset_imu_propagator = false;

  if (measurements.px4_odometry) {
    std::cout << "PX4 ODOM MEASUREMENT RECEIVED" << std::endl;
    total_measurements += ODOM_MEASUREMENTS;
  }
  if (measurements.openvins) {
    std::cout << "OpenVINS ODOM MEASUREMENT RECEIVED" << std::endl;
    total_measurements += ODOM_MEASUREMENTS;
  }
  if (measurements.fast_lio) {
    std::cout << "FastLIO ODOM MEASUREMENT RECEIVED" << std::endl;
    total_measurements += ODOM_MEASUREMENTS;
  }
  if (measurements.imu) {
    std::cout << "IMU MEASUREMENT RECEIVED" << std::endl;
    total_measurements += IMU_MEASUREMENTS;
  }
  if (total_measurements == 0) {
    RCLCPP_DEBUG_STREAM(logger_, "No measurements available. Skipping correction step.");
    return;
  }

  Eigen::VectorXd z = Eigen::VectorXd::Zero(total_measurements);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_measurements, STATES);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_measurements, total_measurements);

  int current_row = 0;
  if (measurements.px4_odometry) {
    RCLCPP_DEBUG_STREAM(logger_, "Processing PX4 Odometry measurement...");
    int start_row = current_row;
    current_row   = processOdometryMeasurement(measurements.px4_odometry.value(), r_gains_.px4_odometry, H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement = true;
      Eigen::Vector3d p_corrected =
          Eigen::Vector3d(measurements.px4_odometry.value().pose.pose.position.x, measurements.px4_odometry.value().pose.pose.position.y,
                          measurements.px4_odometry.value().pose.pose.position.z);
      Eigen::Vector3d v_corrected =
          Eigen::Vector3d(measurements.px4_odometry.value().twist.twist.linear.x, measurements.px4_odometry.value().twist.twist.linear.y,
                          measurements.px4_odometry.value().twist.twist.linear.z);
      Eigen::Quaterniond q_corrected =
          Eigen::Quaterniond(measurements.px4_odometry.value().pose.pose.orientation.w, measurements.px4_odometry.value().pose.pose.orientation.x,
                             measurements.px4_odometry.value().pose.pose.orientation.y, measurements.px4_odometry.value().pose.pose.orientation.z);

      std::cout << "RESETTING IMU PROPAGATOR TO CORRECTED STATE" << std::endl;
      std::cout << "  Position:    " << p_corrected.transpose() << std::endl;
      std::cout << "  Velocity:    " << v_corrected.transpose() << std::endl;
      std::cout << "  Orientation: " << q_corrected.coeffs().transpose() << std::endl;

      imu_propagator_.set_state(p_corrected, v_corrected, q_corrected);

      Eigen::Matrix<double, laser_uav_lib::DIM_ERROR, laser_uav_lib::DIM_ERROR> cov_corrected;
      cov_corrected.setIdentity();
      cov_corrected *= 1e-4;
      imu_propagator_.set_covariance(cov_corrected);

      if (is_debug_) {
        RCLCPP_DEBUG_STREAM(logger_, ">> ImuPropagator RESETTED to corrected state.");
      }
    }
  }

  if (measurements.openvins) {
    RCLCPP_DEBUG_STREAM(logger_, "Processing OpenVINS Odometry measurement...");
    int start_row = current_row;
    current_row   = processOdometryMeasurement(measurements.openvins.value(), r_gains_.openvins, H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement              = true;
      Eigen::Vector3d    p_corrected = Eigen::Vector3d(measurements.openvins.value().pose.pose.position.x, measurements.openvins.value().pose.pose.position.y,
                                                       measurements.openvins.value().pose.pose.position.z);
      Eigen::Vector3d    v_corrected = Eigen::Vector3d(measurements.openvins.value().twist.twist.linear.x, measurements.openvins.value().twist.twist.linear.y,
                                                       measurements.openvins.value().twist.twist.linear.z);
      Eigen::Quaterniond q_corrected =
          Eigen::Quaterniond(measurements.openvins.value().pose.pose.orientation.w, measurements.openvins.value().pose.pose.orientation.x,
                             measurements.openvins.value().pose.pose.orientation.y, measurements.openvins.value().pose.pose.orientation.z);

      std::cout << "RESETTING IMU PROPAGATOR TO CORRECTED STATE" << std::endl;
      std::cout << "  Position:    " << p_corrected.transpose() << std::endl;
      std::cout << "  Velocity:    " << v_corrected.transpose() << std::endl;
      std::cout << "  Orientation: " << q_corrected.coeffs().transpose() << std::endl;

      imu_propagator_.set_state(p_corrected, v_corrected, q_corrected);

      Eigen::Matrix<double, laser_uav_lib::DIM_ERROR, laser_uav_lib::DIM_ERROR> cov_corrected;
      cov_corrected.setIdentity();
      cov_corrected *= 1e-4;
      imu_propagator_.set_covariance(cov_corrected);

      if (is_debug_) {
        RCLCPP_DEBUG_STREAM(logger_, ">> ImuPropagator RESETTED to corrected state.");
      }
    }
  }

  if (measurements.fast_lio) {
    RCLCPP_DEBUG_STREAM(logger_, "Processing FastLIO Odometry measurement...");
    int start_row = current_row;
    current_row   = processOdometryMeasurement(measurements.fast_lio.value(), r_gains_.fast_lio, H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement              = true;
      Eigen::Vector3d    p_corrected = Eigen::Vector3d(measurements.fast_lio.value().pose.pose.position.x, measurements.fast_lio.value().pose.pose.position.y,
                                                       measurements.fast_lio.value().pose.pose.position.z);
      Eigen::Vector3d    v_corrected = Eigen::Vector3d(measurements.fast_lio.value().twist.twist.linear.x, measurements.fast_lio.value().twist.twist.linear.y,
                                                       measurements.fast_lio.value().twist.twist.linear.z);
      Eigen::Quaterniond q_corrected =
          Eigen::Quaterniond(measurements.fast_lio.value().pose.pose.orientation.w, measurements.fast_lio.value().pose.pose.orientation.x,
                             measurements.fast_lio.value().pose.pose.orientation.y, measurements.fast_lio.value().pose.pose.orientation.z);

      std::cout << "RESETTING IMU PROPAGATOR TO CORRECTED STATE" << std::endl;
      std::cout << "  Position:    " << p_corrected.transpose() << std::endl;
      std::cout << "  Velocity:    " << v_corrected.transpose() << std::endl;
      std::cout << "  Orientation: " << q_corrected.coeffs().transpose() << std::endl;

      imu_propagator_.set_state(p_corrected, v_corrected, q_corrected);

      Eigen::Matrix<double, laser_uav_lib::DIM_ERROR, laser_uav_lib::DIM_ERROR> cov_corrected;
      cov_corrected.setIdentity();
      cov_corrected *= 1e-4;
      imu_propagator_.set_covariance(cov_corrected);

      if (is_debug_) {
        RCLCPP_DEBUG_STREAM(logger_, ">> ImuPropagator RESETTED to corrected state.");
      }
    }
  }


  if (measurements.imu && measurements.dt) {
    int start_row = current_row;
    current_row   = processImuMeasurement(measurements.imu.value(), measurements.dt.value(), H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement = true;
    }
    current_row += IMU_MEASUREMENTS;
    RCLCPP_DEBUG_STREAM(logger_, "Adding IMU measurement.");
  }

  if (z.hasNaN()) {
    RCLCPP_WARN_STREAM(logger_, "Combined measurements contain NaN! Correction skipped.");
    return;
  }

  double min_variance = 1e-6;
  for (int i = 0; i < total_measurements; ++i) {
    if (R(i, i) < min_variance)
      R(i, i) = min_variance;
  }

  Eigen::VectorXd z_pred = H * x_;

  Eigen::VectorXd y = z - z_pred;

  if (has_q_measurement) {
    for (int i = 0; i < H.rows(); ++i) {
      if (std::abs(H(i, State::QW) - 1.0) < 1e-9) {
        RCLCPP_DEBUG_STREAM(logger_, "Processing quaternion measurement found at row: " << i);

        const int q_start_row = i;

        Eigen::Quaterniond q_z(z(q_start_row), z(q_start_row + 1), z(q_start_row + 2), z(q_start_row + 3));

        Eigen::Quaterniond q_z_pred(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));

        q_z.normalize();
        q_z_pred.normalize();

        if (q_z.coeffs().dot(q_z_pred.coeffs()) < 0.0) {
          q_z.coeffs() *= -1.0;
        }

        Eigen::Quaterniond q_error = q_z * q_z_pred.inverse();

        Eigen::AngleAxisd error_angle_axis(q_error);
        Eigen::Vector3d   attitude_error_3d = error_angle_axis.axis() * error_angle_axis.angle();

        y.segment<3>(q_start_row + 1) = attitude_error_3d;
        y(q_start_row)                = 0;

        i += 3;
      }
    }
  }

  Eigen::MatrixXd S = H * P_ * H.transpose() + R;

  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  Eigen::VectorXd correction = K * y;

  Eigen::Quaterniond q_old(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
  Eigen::Vector3d    attitude_correction_3d = correction.segment<3>(State::QX);

  Eigen::Quaterniond q_correction;
  double             angle = attitude_correction_3d.norm();
  if (angle > 0.0) {
    Eigen::Vector3d axis = attitude_correction_3d.normalized();
    q_correction         = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  } else {
    q_correction.setIdentity();
  }
  Eigen::Quaterniond q_new = q_correction * q_old;
  x_(State::QW)            = q_new.normalized().w();
  x_(State::QX)            = q_new.normalized().x();
  x_(State::QY)            = q_new.normalized().y();
  x_(State::QZ)            = q_new.normalized().z();

  x_.segment<3>(State::PX) += correction.segment<3>(State::PX);
  x_.segment<3>(State::VX) += correction.segment<3>(State::VX);
  x_.segment<3>(State::WX) += correction.segment<3>(State::WX);

  x_old_ = x_;

  Eigen::Matrix<double, STATES, STATES> I;
  I.setIdentity();
  P_ = (I - K * H) * P_;

  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "State (x) AFTER:");
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "Innovation (norm y): " << y.norm());
    RCLCPP_DEBUG_STREAM(logger_, "Innovation (y): " << y.transpose());
    RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) AFTER: " << P_.trace());

    RCLCPP_DEBUG_STREAM(logger_, "--- Confiança do Filtro (Norma do Ganho de Kalman K) ---");
    int print_row = 0;

    if (measurements.px4_odometry) {
      double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
      RCLCPP_DEBUG_STREAM(logger_, "  ├─ PX4 Odom: " << norm);
      print_row += ODOM_MEASUREMENTS;
    }
    if (measurements.openvins) {
      double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
      RCLCPP_DEBUG_STREAM(logger_, "  ├─ OpenVINS: " << norm);
      print_row += ODOM_MEASUREMENTS;
    }
    if (measurements.fast_lio) {
      double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
      RCLCPP_DEBUG_STREAM(logger_, "  ├─ FastLIO:  " << norm);
      print_row += ODOM_MEASUREMENTS;
    }
    if (measurements.imu) {
      double norm = K.block(0, print_row, STATES, IMU_MEASUREMENTS).norm();
      RCLCPP_DEBUG_STREAM(logger_, "  ├─ IMU (W):  " << norm);
      print_row += IMU_MEASUREMENTS;
    }
  }

  std::cout << "Corrected State after EKF:" << std::endl;
  std::cout << "  Position:    " << x_.segment<3>(State::PX).transpose() << std::endl;
  std::cout << "  Velocity:    " << x_.segment<3>(State::VX).transpose() << std::endl;
  std::cout << "  Orientation: " << Eigen::Quaterniond(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ)).coeffs().transpose() << std::endl;
  std::cout << "  Ang. Vel.:   " << x_.segment<3>(State::WX).transpose() << std::endl;

  x_(State::PX) = pos_x_filter_.iterate(x_(State::PX));
  x_(State::PY) = pos_y_filter_.iterate(x_(State::PY));
  x_(State::PZ) = pos_z_filter_.iterate(x_(State::PZ));

  x_(State::VX) = vel_x_filter_.iterate(x_(State::VX));
  x_(State::VY) = vel_y_filter_.iterate(x_(State::VY));
  x_(State::VZ) = vel_z_filter_.iterate(x_(State::VZ));

  x_(State::WX) = ang_vel_x_filter_.iterate(x_(State::WX));
  x_(State::WY) = ang_vel_y_filter_.iterate(x_(State::WY));
  x_(State::WZ) = ang_vel_z_filter_.iterate(x_(State::WZ));

  std::cout << "Corrected State after filter:" << std::endl;
  std::cout << "  Position:    " << x_.segment<3>(State::PX).transpose() << std::endl;
  std::cout << "  Velocity:    " << x_.segment<3>(State::VX).transpose() << std::endl;
  std::cout << "  Orientation: " << Eigen::Quaterniond(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ)).coeffs().transpose() << std::endl;
  std::cout << "  Ang. Vel.:   " << x_.segment<3>(State::WX).transpose() << std::endl;
}
//}

/* processOdometryMeasurement() //{ */
int StateEstimator::processOdometryMeasurement(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains, Eigen::MatrixXd &H, Eigen::VectorXd &z,
                                               Eigen::MatrixXd &R, int current_row) {

  Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());

  if (!std::isnan(odom.pose.pose.position.x)) {
    constexpr int POS_MEASUREMENTS = 3;
    H.block<POS_MEASUREMENTS, POS_MEASUREMENTS>(current_row, State::PX).setIdentity();
    z.segment<POS_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    R.block<3, 3>(current_row, current_row) = Eigen::Matrix3d::Identity() * gains.position;
    current_row += POS_MEASUREMENTS;
  }

  if (!std::isnan(odom.pose.pose.orientation.w)) {
    constexpr int ORI_MEASUREMENTS = 4;
    H.block<ORI_MEASUREMENTS, ORI_MEASUREMENTS>(current_row, State::QW).setIdentity();
    z.segment<ORI_MEASUREMENTS>(current_row) << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z;

    R.block<3, 3>(current_row + 1, current_row + 1) = Eigen::Matrix3d::Identity() * gains.orientation;
    R(current_row, current_row)                     = 0.1;

    current_row += ORI_MEASUREMENTS;
  }

  if (!std::isnan(odom.twist.twist.linear.x)) {
    constexpr int VEL_MEASUREMENTS = 3;
    H.block<VEL_MEASUREMENTS, VEL_MEASUREMENTS>(current_row, State::VX).setIdentity();
    z.segment<VEL_MEASUREMENTS>(current_row) << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
    R.block<3, 3>(current_row, current_row) = Eigen::Matrix3d::Identity() * gains.linear_velocity;
    current_row += VEL_MEASUREMENTS;
  }

  if (!std::isnan(odom.twist.twist.angular.x)) {
    constexpr int ANG_MEASUREMENTS = 3;
    H.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, State::WX).setIdentity();
    z.segment<ANG_MEASUREMENTS>(current_row) << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
    R.block<3, 3>(current_row, current_row) = Eigen::Matrix3d::Identity() * gains.angular_velocity;
    current_row += ANG_MEASUREMENTS;
  }

  return current_row;
}
//}

/* processImuMeasurement() //{ */
int StateEstimator::processImuMeasurement(const sensor_msgs::msg::Imu &imu, double dt, Eigen::MatrixXd &H, Eigen::VectorXd &z, Eigen::MatrixXd &R,
                                          int current_row) {
  std::cout << "Processing IMU Measurement with dt = " << dt << std::endl;
  std::cout << "  Angular Velocity: "
            << "[" << imu.angular_velocity.x << ", " << imu.angular_velocity.y << ", " << imu.angular_velocity.z << "]" << std::endl;
  std::cout << "  Linear Acceleration: "
            << "[" << imu.linear_acceleration.x << ", " << imu.linear_acceleration.y << ", " << imu.linear_acceleration.z << "]" << std::endl;

  imu_propagator_.propagate(Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z),
                            Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z), dt);

  std::cout << "  IMU Propagated State:" << std::endl;
  Eigen::Vector3d    p_imu = imu_propagator_.get_position();
  Eigen::Vector3d    v_imu = imu_propagator_.get_velocity();
  Eigen::Quaterniond q_imu = imu_propagator_.get_orientation();
  std::cout << "State: " << std::endl;
  std::cout << "  ├  Position : " << x_.segment<3>(State::PX).transpose() << std::endl;
  std::cout << "  ├  Velocity : " << x_.segment<3>(State::VX).transpose() << std::endl;
  std::cout << "  ├  Orientation : " << Eigen::Quaterniond(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ)).coeffs().transpose() << std::endl;

  std::cout << "propagation: " << std::endl;
  std::cout << "  ├  Position : " << p_imu.transpose() << std::endl;
  std::cout << "  ├  Velocity : " << v_imu.transpose() << std::endl;
  std::cout << "  ├  Orientation : " << q_imu.coeffs().transpose() << std::endl;

  // constexpr int VEL_MEASUREMENTS = 3;
  // H.block<VEL_MEASUREMENTS, VEL_MEASUREMENTS>(current_row, State::VX).setIdentity();
  // Eigen::Vector3d imu_vel = imu_propagator_.get_velocity();
  // z.segment<VEL_MEASUREMENTS>(current_row) << imu_vel.x(), imu_vel.y(), imu_vel.z();
  // R.block<VEL_MEASUREMENTS, VEL_MEASUREMENTS>(current_row, current_row) = Eigen::Matrix3d::Identity() * r_gains_.imu.linear_velocity;  // Ex: 0.05
  // current_row += VEL_MEASUREMENTS;

  // constexpr int QUAT_MEASUREMENTS = 4;
  // H.block<QUAT_MEASUREMENTS, QUAT_MEASUREMENTS>(current_row, State::QW).setIdentity();
  // Eigen::Quaterniond imu_q = imu_propagator_.get_orientation();
  // z.segment<QUAT_MEASUREMENTS>(current_row) << imu_q.w(), imu_q.x(), imu_q.y(), imu_q.z();
  // R.block<QUAT_MEASUREMENTS, QUAT_MEASUREMENTS>(current_row, current_row) = Eigen::Matrix4d::Identity() * r_gains_.imu.orientation;  // Ex: 0.01
  // current_row += QUAT_MEASUREMENTS;

  constexpr int ANG_MEASUREMENTS = 3;
  H.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, State::WX).setIdentity();
  Eigen::Vector3d gyro_raw(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  Eigen::Vector3d gyro_corrected = gyro_raw - imu_propagator_.get_bias_gyr();
  z.segment<ANG_MEASUREMENTS>(current_row) << gyro_corrected.x(), gyro_corrected.y(), gyro_corrected.z();
  R.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, current_row) = Eigen::Matrix3d::Identity() * r_gains_.imu.angular_velocity;  // Ex: 0.001
  current_row += ANG_MEASUREMENTS;

  return current_row;
}
//}

/* calculate_custom_attitude_error() //{ */
Eigen::Vector3d StateEstimator::calculate_custom_attitude_error(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_ref) {
  Eigen::Quaterniond q_error = q * q_ref.inverse();

  double w_e = q_error.w();
  double x_e = q_error.x();
  double y_e = q_error.y();
  double z_e = q_error.z();

  double q_att_denom = std::sqrt(w_e * w_e + z_e * z_e + 1e-3);

  Eigen::Vector3d q_att_num;
  q_att_num << w_e * x_e - y_e * z_e, w_e * y_e + x_e * z_e, z_e;

  Eigen::Vector3d q_att = q_att_num / q_att_denom;

  return q_att;
}
//}

/* IntegrateVelocityRK4() //{ */
Eigen::Vector3d StateEstimator::IntegrateVelocityRK4(const Eigen::Vector3d &current_velocity, const Eigen::Vector3d &linear_acceleration, double delta_t) {
  auto system = [linear_acceleration](const std::vector<double> &v, std::vector<double> &dvdt, double t) {
    dvdt[0] = linear_acceleration.x();
    dvdt[1] = linear_acceleration.y();
    dvdt[2] = linear_acceleration.z();
  };

  std::vector<double> velocity_state = {current_velocity.x(), current_velocity.y(), current_velocity.z()};

  boost::numeric::odeint::runge_kutta4<std::vector<double>> rk4;

  rk4.do_step(system, velocity_state, 0.0, delta_t);

  return Eigen::Vector3d(velocity_state[0], velocity_state[1], velocity_state[2]);
}
//}

}  // namespace laser_uav_estimators
