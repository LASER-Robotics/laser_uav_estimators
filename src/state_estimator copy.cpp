#include <laser_uav_estimators/state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace laser_uav_estimators
{
/* StateEstimator() //{ */
StateEstimator::StateEstimator(const double &mass, const Eigen::MatrixXd &allocation_matrix, const Eigen::Matrix3d &inertia, const std::string &verbosity)
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

  // 1. Extração e Normalização
  Eigen::Quaternion<T> q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
  q.normalize();

  // v_world agora representa a velocidade no referencial inercial (como no Python dot_p = v)
  Eigen::Matrix<T, 3, 1>      v_world = x.template segment<3>(State::VX);
  Eigen::Matrix<T, 3, 1>      w_body  = x.template segment<3>(State::WX);
  Eigen::Matrix<T, STATES, 1> x_dot;

  // 2. Derivada da Posição (Mudar de: q * v_body -> Para: v_world)
  x_dot.template segment<3>(State::PX) = v_world;

  // 3. Derivada do Quatérnio (Corrigido para q * w)
  Eigen::Quaternion<T> w_quat(T(0), w_body.x(), w_body.y(), w_body.z());
  Eigen::Quaternion<T> q_dot_quat      = q * w_quat;
  x_dot.template segment<4>(State::QW) = T(0.5) * Eigen::Matrix<T, 4, 1>(q_dot_quat.w(), q_dot_quat.x(), q_dot_quat.y(), q_dot_quat.z());

  // 4. Forças e Torques (Alocação)
  Eigen::Matrix<T, 4, 1> wrench       = allocation_matrix_.template cast<T>() * u;
  T                      total_thrust = wrench(0);
  Eigen::Matrix<T, 3, 1> tau          = wrench.template segment<3>(1);

  // 5. Derivada da Velocidade Linear (Alinhando com o Acados)
  // No Python: rotate_quaternion(q, [0,0,T/m]) + g
  Eigen::Matrix<T, 3, 1> thrust_body(T(0), T(0), total_thrust / T(mass_));
  Eigen::Matrix<T, 3, 1> g_inertial(T(0), T(0), T(-GRAVITY));  // Use o mesmo valor do Python

  // Aceleração no mundo: Rotacionar empuxo para o mundo e somar gravidade direta
  // Removemos o termo -w.cross(v) pois ele só existe em referenciais não-inerciais (Body)
  x_dot.template segment<3>(State::VX) = (q * thrust_body) + g_inertial;

  // 6. Derivada da Velocidade Angular (Dinâmica de Euler)
  x_dot.template segment<3>(State::WX) = inertia_tensor_inv_.template cast<T>() * (tau - w_body.cross(inertia_tensor_.template cast<T>() * w_body));

  return x_dot;
}
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

  // x_.segment<3>(State::PX) = x_.segment<3>(State::PX) + x_dot.segment<3>(State::PX) * dt;
  // x_.segment<4>(State::QW) = x_.segment<4>(State::QW) + x_dot.segment<4>(State::QW) * dt;
  // x_.segment<3>(State::VX) = x_.segment<3>(State::VX) + x_dot.segment<3>(State::VX) * dt;
  // x_.segment<3>(State::WX) = x_.segment<3>(State::WX) + x_dot.segment<3>(State::WX) * dt;

  x_.segment<4>(State::QW).normalize();

  x_dot = x_dot * dt;

  if (is_debug_) {
    RCLCPP_DEBUG_STREAM(logger_, "State (x_dot) AFTER:");
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_dot.segment<3>(State::PX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_dot.segment<4>(State::QW).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_dot.segment<3>(State::VX).transpose());
    RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_dot.segment<3>(State::WX).transpose());
  }

  if (is_debug_) {
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

  bool has_q_measurement = false;

  if (measurements.px4_odometry)
    total_measurements += ODOM_MEASUREMENTS;
  if (measurements.openvins)
    total_measurements += ODOM_MEASUREMENTS;
  if (measurements.fast_lio)
    total_measurements += ODOM_MEASUREMENTS;
  if (measurements.imu)
    total_measurements += IMU_MEASUREMENTS;
  if (measurements.gps)
    total_measurements += GPS_MEASUREMENTS;

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
    }
  }

  if (measurements.openvins) {
    RCLCPP_DEBUG_STREAM(logger_, "Processing OpenVINS Odometry measurement...");
    int start_row = current_row;
    current_row   = processOdometryMeasurement(measurements.openvins.value(), r_gains_.openvins, H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement = true;
    }
  }

  if (measurements.fast_lio) {
    RCLCPP_DEBUG_STREAM(logger_, "Processing FastLIO Odometry measurement...");
    int start_row = current_row;
    current_row   = processOdometryMeasurement(measurements.fast_lio.value(), r_gains_.fast_lio, H, z, R, current_row);
    if (current_row > start_row) {
      has_q_measurement = true;
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
    if (measurements.gps) {
      double norm = K.block(0, print_row, STATES, GPS_MEASUREMENTS).norm();
      RCLCPP_DEBUG_STREAM(logger_, "  └─ GPS (XY): " << norm);
      print_row += GPS_MEASUREMENTS;
    }
  }
}
//}

/* processOdometryMeasurement() //{ */
/* processOdometryMeasurement() //{ */
int StateEstimator::processOdometryMeasurement(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains,
                                               const Eigen::Matrix<double, STATES, 1> &x,  // Passamos o estado atual
                                               Eigen::MatrixXd &H, Eigen::VectorXd &z, Eigen::MatrixXd &R, int current_row) {

  Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());

  // --- 1. Recuperar o quatérnio de estado atual (do filtro, não da medição) ---
  Eigen::Quaterniond state_q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
  state_q.normalize();

  // --- POSIÇÃO (Referencial Global) ---
  if (!std::isnan(odom.pose.pose.position.x)) {
    constexpr int POS_MEASUREMENTS = 3;
    H.block<POS_MEASUREMENTS, POS_MEASUREMENTS>(current_row, State::PX).setIdentity();
    z.segment<POS_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    R.block<3, 3>(current_row, current_row) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * gains.position;
    current_row += POS_MEASUREMENTS;
  }

  // --- ORIENTAÇÃO (Referencial Global) ---
  if (!std::isnan(odom.pose.pose.orientation.w)) {
    constexpr int ORI_MEASUREMENTS = 4;
    H.block<ORI_MEASUREMENTS, ORI_MEASUREMENTS>(current_row, State::QW).setIdentity();
    z.segment<ORI_MEASUREMENTS>(current_row) << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z;

    R.block<3, 3>(current_row + 1, current_row + 1) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * gains.orientation;
    R(current_row, current_row)                     = 0.1;  // Incerteza do componente W
    current_row += ORI_MEASUREMENTS;
  }

  // --- VELOCIDADE LINEAR (Medição no Body Frame -> Estado no World Frame) ---
  if (!std::isnan(odom.twist.twist.linear.x)) {
    constexpr int VEL_MEASUREMENTS = 3;

    // A medição (z) é mantida no referencial do robô (Body Frame)
    z.segment<VEL_MEASUREMENTS>(current_row) << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;

    // A matriz H mapeia a velocidade do mundo para o corpo: h(x) = R(q)^T * v_world
    // A Jacobiana em relação a v_world é R_world_to_body (ou seja, R.transpose())
    Eigen::Matrix3d R_world_to_body                                     = state_q.toRotationMatrix().transpose();
    H.block<VEL_MEASUREMENTS, VEL_MEASUREMENTS>(current_row, State::VX) = R_world_to_body;

    // A covariância R permanece no referencial do sensor (Body Frame)
    R.block<3, 3>(current_row, current_row) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * gains.linear_velocity;

    current_row += VEL_MEASUREMENTS;
  }

  // --- VELOCIDADE ANGULAR (Referencial do Corpo) ---
  if (!std::isnan(odom.twist.twist.angular.x)) {
    constexpr int ANG_MEASUREMENTS = 3;
    // Como tanto a medição quanto o estado (WX) estão no Body Frame, H é identidade
    H.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, State::WX).setIdentity();
    z.segment<ANG_MEASUREMENTS>(current_row) << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
    R.block<3, 3>(current_row, current_row) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * gains.angular_velocity;
    current_row += ANG_MEASUREMENTS;
  }

  return current_row;
}
//}

/* processImuMeasurement() //{ */
int StateEstimator::processImuMeasurement(const sensor_msgs::msg::Imu &imu, double dt, Eigen::MatrixXd &H, Eigen::VectorXd &z, Eigen::MatrixXd &R,
                                          int current_row) {

  // --- VELOCIDADE ANGULAR ---
  if (!std::isnan(imu.angular_velocity.x)) {
    constexpr int ANG_MEASUREMENTS = 3;
    H.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, State::WX).setIdentity();
    z.segment<ANG_MEASUREMENTS>(current_row) << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;

    R.block<ANG_MEASUREMENTS, ANG_MEASUREMENTS>(current_row, current_row) = Eigen::Matrix3d::Identity() * r_gains_.imu.angular_velocity;
    current_row += ANG_MEASUREMENTS;
  }

  // --- ACELERAÇÃO LINEAR (Como medição de velocidade no Mundo) ---
  if (!std::isnan(imu.linear_acceleration.x)) {
    // 1. Recuperar o estado atual (World Frame)
    Eigen::Quaterniond q_current(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
    q_current.normalize();
    Eigen::Vector3d v_world_prev = x_.segment<3>(State::VX);

    // 2. Aceleração bruta da IMU (Body Frame)
    Eigen::Vector3d linear_accel_meas(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);

    // 3. Compensação da Gravidade no World Frame
    // Lógica: a_world = (R * a_body) + g_world
    // Onde g_world = [0, 0, -9.806]
    Eigen::Vector3d gravity_world(0.0, 0.0, -GRAVITY);
    Eigen::Vector3d linear_accel_world = (q_current * linear_accel_meas) + gravity_world;

    // 4. Integração para gerar a "medição" de velocidade global
    // Usando uma integração simples (Euler) condizente com o passo dt da IMU
    Eigen::Vector3d v_world_measured = v_world_prev + (linear_accel_world * dt);

    // 5. Matriz de Observação H
    // Como v_world_measured está no World Frame, ele mapeia direto para o estado VX (Identity)
    constexpr int ACC_MEASUREMENTS = 3;
    H.block<ACC_MEASUREMENTS, ACC_MEASUREMENTS>(current_row, State::VX).setIdentity();

    // O vetor z recebe a velocidade integrada
    z.segment<ACC_MEASUREMENTS>(current_row) = v_world_measured;

    // 6. Covariância R
    R.block<ACC_MEASUREMENTS, ACC_MEASUREMENTS>(current_row, current_row) = Eigen::Matrix3d::Identity() * r_gains_.imu.linear_velocity;

    current_row += ACC_MEASUREMENTS;
  }

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
