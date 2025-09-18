/**
 * @file state_estimator.cpp
 * @brief Implementation of the StateEstimator class methods.
 * @author Wagner Dantas Garcia / Laser UAV Team
 * @date September 10, 2025
 */
#include <laser_uav_estimators/state_estimator.hpp>
#include <rclcpp/logging.hpp>
#include <sstream> // Required for std::stringstream

namespace laser_uav_estimators
{
    /**
     * @brief Constructor for the StateEstimator class.
     * @param mass The drone's mass in kg.
     * @param motor_positions The 4x2 matrix with the (x,y) positions of the motors in meters.
     * @param thrust_coeff The thrust coefficient of the motors.
     * @param torque_coeff The torque coefficient of the motors.
     * @param inertia The 3x3 inertia matrix of the drone.
     * @param verbosity The logging level for debugging ("SILENT", "INFO", "DEBUG").
     */
    StateEstimator::StateEstimator(const double &mass, const Eigen::Matrix<double, 4, 2> &motor_positions, const double &thrust_coeff, const double &torque_coeff, const Eigen::Matrix3d &inertia, const std::string &verbosity)
        : mass_(mass),
          motor_positions_(motor_positions),
          thrust_coefficient_(thrust_coeff),
          torque_coefficient_(torque_coeff),
          inertia_tensor_(inertia),
          logger_(rclcpp::get_logger("StateEstimator")),
          q_gains_(),
          r_gains_()
    {
        // Set the verbosity level and the corresponding debug flag.
        set_verbosity(verbosity);
        RCLCPP_INFO(logger_, "--- STATE ESTIMATOR CONSTRUCTOR ---");

        // Pre-calculate the inverse of the inertia matrix for runtime optimization.
        inertia_tensor_inv_ = inertia.inverse();

        // --- Initialize State (x_) ---
        // Set the entire state vector to zero.
        x_.setZero();
        // Define the initial orientation as an identity quaternion (no rotation).
        x_(State::QW) = 1.0;

        // --- Initialize Covariance (P_) ---
        // Start with an identity matrix, representing equal uncertainty in all states.
        P_.setIdentity();
        P_ *= 0.1; // Set a moderate initial uncertainty.

        // --- Initialize Noise and Jacobian Matrices ---
        F_.setIdentity();  // The Jacobian F is initialized as identity.
        Q_.setIdentity();  // Process noise covariance.
        update_Q_matrix(); // Build the Q matrix with default low process noise.

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Output: Initial state x_ = ");
            RCLCPP_DEBUG_STREAM(logger_, "     ├ Pos:          " << x_.template segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     ├ Quat:         " << x_.template segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     ├ Lin. Vel.:    " << x_.template segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     └ Ang. Vel.:    " << x_.template segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Output: Initial uncertainty (trace P) = " << P_.trace());
        }
    }

    /**
     * @brief Sets the process (model) noise gains.
     * @param gains The struct with the new gains.
     */
    void StateEstimator::set_process_noise_gains(const ProcessNoiseGains &gains)
    {
        RCLCPP_INFO(logger_, "Updating process noise gains (Q).");
        q_gains_ = gains;
        update_Q_matrix(); // Rebuild the Q matrix with the new gains
    }

    /**
     * @brief Sets the measurement (sensor) noise gains.
     * @param gains The struct with the new gains.
     */
    void StateEstimator::set_measurement_noise_gains(const MeasurementNoiseGains &gains)
    {
        RCLCPP_INFO(logger_, "Updating measurement noise gains (R).");
        r_gains_ = gains;
    }

    /**
     * @brief Updates the process noise covariance matrix (Q)
     * based on the current values in q_gains_.
     */
    void StateEstimator::update_Q_matrix()
    {
        Q_.setZero();
        // Set diagonal blocks corresponding to each part of the state vector.
        Q_.block<3, 3>(State::PX, State::PX) = Eigen::Matrix3d::Identity() * q_gains_.position;
        Q_.block<4, 4>(State::QW, State::QW) = Eigen::Matrix4d::Identity() * q_gains_.orientation;
        Q_.block<3, 3>(State::VX, State::VX) = Eigen::Matrix3d::Identity() * q_gains_.linear_velocity;
        Q_.block<3, 3>(State::WX, State::WX) = Eigen::Matrix3d::Identity() * q_gains_.angular_velocity;

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Process noise matrix Q updated. Trace: " << Q_.trace());
        }
    }

    /**
     * @brief Resets the filter's state and covariance to their initial values.
     * Useful for re-initializing the estimate after a failure or when the drone is in a known state.
     */
    void StateEstimator::reset()
    {
        RCLCPP_INFO(logger_, "--- EKF FILTER RESET ---");

        // Reset the state to the origin, with no rotation and zero velocities.
        x_.setZero();
        x_(State::QW) = 1.0;

        // Reset the uncertainty to its initial value.
        P_.setIdentity();
        P_ *= 0.1;

        // Re-initialize the transition and noise matrices.
        F_.setIdentity();
        update_Q_matrix();

        RCLCPP_INFO(logger_, "State and uncertainty reset to default values.");
    }

    /**
     * @brief Sets the verbosity level of the logs.
     * @param verbosity The verbosity string ("ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT").
     */
    void StateEstimator::set_verbosity(const std::string &verbosity)
    {
        verbosity_ = verbosity;
        // Update the boolean flag to optimize real-time log checks.
        is_debug_ = (verbosity_ == "DEBUG" || verbosity_ == "ALL");

        // Configure the ROS2 logger's level based on the provided string.
        if (verbosity_ == "SILENT")
        {
            logger_.set_level(rclcpp::Logger::Level::Fatal);
        }
        else if (verbosity_ == "ERROR")
        {
            logger_.set_level(rclcpp::Logger::Level::Error);
        }
        else if (verbosity_ == "WARNING")
        {
            logger_.set_level(rclcpp::Logger::Level::Warn);
        }
        else if (is_debug_)
        { // "ALL" or "DEBUG"
            logger_.set_level(rclcpp::Logger::Level::Debug);
        }
        else
        { // Default is "INFO"
            logger_.set_level(rclcpp::Logger::Level::Info);
        }

        RCLCPP_INFO_STREAM(logger_, "Verbosity level set to: " << verbosity_);
    }

    /**
     * @brief Gets the current verbosity level.
     * @return std::string The verbosity level.
     */
    std::string StateEstimator::get_verbosity() const
    {
        return verbosity_;
    }

    /**
     * @brief The drone's non-linear state transition model.
     * @details Implements the differential equations describing the physics of the drone's motion.
     * It is a template method to be used with `autodiff::real` for automatic differentiation.
     * @param x The current state vector.
     * @param u The applied control vector.
     * @return The vector of state derivatives (x_dot).
     */
    template <typename T>
    Eigen::Matrix<T, STATES, 1> StateEstimator::state_transition_model(
        const Eigen::Matrix<T, STATES, 1> &x,
        const Eigen::Matrix<T, INPUTS, 1> &u) const
    {
        // --- 1. Extract Variables from the State Vector ---
        Eigen::Quaternion<T> q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
        q.normalize(); // Ensure the quaternion represents a valid rotation.
        Eigen::Matrix<T, 3, 1> v_body = x.template segment<3>(State::VX);
        Eigen::Matrix<T, 3, 1> w_body = x.template segment<3>(State::WX);
        Eigen::Matrix<T, STATES, 1> x_dot; // Vector to store the derivatives.

        // --- 2. Kinematics: Position and Orientation Derivatives ---
        // The derivative of position is the linear velocity rotated into the inertial frame.
        x_dot.template segment<3>(State::PX) = q.toRotationMatrix() * v_body;

        // The derivative of the quaternion is calculated based on the angular velocity.
        Eigen::Quaternion<T> w_quat(T(0), w_body.x(), w_body.y(), w_body.z());
        Eigen::Quaternion<T> q_dot_quat = q * w_quat;
        x_dot.template segment<4>(State::QW) = T(0.5) * Eigen::Matrix<T, 4, 1>(
                                                            q_dot_quat.w(), q_dot_quat.x(), q_dot_quat.y(), q_dot_quat.z());

        // --- 3. Dynamics: Velocity Derivatives ---
        // The total thrust force acts along the drone's body Z-axis.
        T total_thrust = u.sum();
        Eigen::Matrix<T, 3, 1> thrust_force_body(T(0), T(0), total_thrust);

        // Gravity is defined in the inertial frame and must be rotated into the body frame.
        Eigen::Matrix<T, 3, 1> g_inertial(T(0), T(0), T(-GRAVITY));
        Eigen::Matrix<T, 3, 1> g_body = q.toRotationMatrix().transpose() * g_inertial;

        // Linear acceleration (Newton's 2nd Law): a = (F_thrust/m) + g - w x v
        x_dot.template segment<3>(State::VX) = (thrust_force_body / T(mass_)) + g_body - w_body.cross(v_body);

        // Torques are generated based on the position of each motor.
        Eigen::Matrix<T, 3, 1> tau;
        tau << (u(0) * T(motor_positions_(0, 1)) + u(1) * T(motor_positions_(1, 1)) + u(2) * T(motor_positions_(2, 1)) + u(3) * T(motor_positions_(3, 1))), // Roll torque (X-axis)
            (-u(0) * T(motor_positions_(0, 0)) - u(1) * T(motor_positions_(1, 0)) - u(2) * T(motor_positions_(2, 0)) - u(3) * T(motor_positions_(3, 0))),   // Pitch torque (Y-axis)
            T(torque_coefficient_) * (-u(0) - u(1) + u(2) + u(3));                                                                                          // Yaw torque (Z-axis)

        // Angular acceleration (Euler's equation): α = I⁻¹ * (τ - w x (I * w))
        x_dot.template segment<3>(State::WX) = inertia_tensor_inv_.template cast<T>() * (tau - w_body.cross(inertia_tensor_.template cast<T>() * w_body));

        return x_dot;
    }

    /**
     * @brief Calculates the Jacobian matrix F (derivative of the state transition model w.r.t. the state).
     * It uses automatic differentiation to avoid manual calculation.
     * @param x The current state vector.
     * @param u The current control vector.
     */
    void StateEstimator::calculate_jacobian_F(
        const Eigen::Matrix<double, STATES, 1> &x,
        const Eigen::Matrix<double, INPUTS, 1> &u)
    {
        // Convert the input vectors to the `autodiff::real` type for differentiation.
        autodiff::VectorXreal x_ad = x;
        autodiff::VectorXreal u_ad = u;

        // Create a lambda function representing the discretized state model: x_k+1 = x_k + f(x,u)*dt.
        auto model_for_autodiff = [&](const autodiff::VectorXreal &x_arg) -> autodiff::VectorXreal
        {
            return x_arg + this->state_transition_model<autodiff::real>(x_arg, u_ad.cast<autodiff::real>()) * this->dt_;
        };

        // The `autodiff` library calculates the Jacobian of the lambda function with respect to the state `x_ad`.
        F_ = autodiff::jacobian(model_for_autodiff, wrt(x_ad), at(x_ad));
    }

    /**
     * @brief Executes the EKF prediction step.
     * @details Projects the current state and covariance to the next time step.
     * @param u The control vector (forces of the 4 motors).
     * @param dt The time interval (delta t) since the last prediction, in seconds.
     */
    void StateEstimator::predict(const Eigen::Matrix<double, INPUTS, 1> &u, double dt)
    {
        RCLCPP_DEBUG_STREAM(logger_, "--- PREDICT ---");
        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Inputs: u = " << u.transpose() << ", dt = " << dt);
            RCLCPP_DEBUG_STREAM(logger_, "State (x) BEFORE:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) BEFORE: " << P_.trace());
        }

        dt_ = dt; // Store dt for use in the Jacobian calculation.

        // 1. Linearize the motion model at the current point to get F_.
        calculate_jacobian_F(x_, u);

        // 2. Propagate the uncertainty (covariance) to the next step: P = F * P * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;

        // 3. Propagate the state using the non-linear model: x = x + x_dot * dt
        Eigen::Matrix<double, STATES, 1> x_dot = state_transition_model<double>(x_, u);
        x_ += x_dot * dt;

        // 4. Normalize the quaternion to ensure it remains a valid rotation.
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "State (x) AFTER:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) AFTER: " << P_.trace());
        }
    }

    /**
     * @brief Executes the EKF correction step using available measurements.
     * @details This method acts as a dispatcher, batching all available sensor data
     * into a single large update step.
     * @param measurements A package containing the available sensor data.
     */
    void StateEstimator::correct(const MeasurementPackage &measurements)
    {
        RCLCPP_DEBUG_STREAM(logger_, "--- CORRECT ---");

        // --- STEP 1: Pre-calculate the total size of the measurement vector ---
        // This is crucial for initializing matrices with the correct size to avoid errors.
        int total_measurements = 0;
        constexpr int ODOM_MEASUREMENTS = 13; // P, Q, V, W
        constexpr int IMU_MEASUREMENTS = 3;   // Angular velocity only
        constexpr int GPS_MEASUREMENTS = 2;   // XY Position

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

        // If there are no measurements, there's nothing to do.
        if (total_measurements == 0)
        {
            RCLCPP_DEBUG_STREAM(logger_, "No measurements available. Skipping correction step.");
            return;
        }

        // --- STEP 2: Initialize the combined matrices with the final size and ZEROED out ---
        // This ensures that off-diagonal blocks in R are zero, which is correct
        // for uncorrelated sensor noise.
        Eigen::VectorXd z = Eigen::VectorXd::Zero(total_measurements);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_measurements, STATES);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_measurements, total_measurements);

        int current_row = 0; // Pointer to the current row where data will be inserted.

        // --- STEP 3: Fill the matrices in blocks, safely ---

        // Process PX4 Odometry (if available)
        if (measurements.px4_odometry)
        {
            const auto &odom = measurements.px4_odometry.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();

            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

            // Build the R matrix from the message's covariance and our custom gains
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1; // Error in W component is not used/meaningful in 3D error space
            has_q_measurement = true;
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adding PX4 Odometry measurement.");
        }

        // Process OpenVINS Odometry (if available)
        if (measurements.openvins)
        {
            // Logic is identical to PX4, just with a different data source
            const auto &odom = measurements.openvins.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();
            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.openvins.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.openvins.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.openvins.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.openvins.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1;
            has_q_measurement = true;
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adding OpenVINS Odometry measurement.");
        }

        // Process FastLIO Odometry (if available)
        if (measurements.fast_lio)
        {
            // Identical logic
            const auto &odom = measurements.fast_lio.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();
            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1;
            has_q_measurement = true;
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adding FastLIO Odometry measurement.");
        }

        // Process IMU (if available)
        if (measurements.imu)
        {
            const auto &imu = measurements.imu.value();
            // The IMU measures angular velocity, which corresponds to states WX, WY, WZ.
            H.block<IMU_MEASUREMENTS, IMU_MEASUREMENTS>(current_row, State::WX).setIdentity();
            z.segment<IMU_MEASUREMENTS>(current_row) << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;
            R.block<IMU_MEASUREMENTS, IMU_MEASUREMENTS>(current_row, current_row) = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(imu.angular_velocity_covariance.data()) + Eigen::Matrix3d::Identity() * r_gains_.imu.angular_velocity;

            current_row += IMU_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adding IMU (Angular Vel.) measurement.");
        }

        // Process GPS (if available)
        if (measurements.gps)
        {
            const auto &gps = measurements.gps.value();
            // GPS measures position PX and PY.
            H(current_row, State::PX) = 1.0;
            H(current_row + 1, State::PY) = 1.0;
            z.segment<GPS_MEASUREMENTS>(current_row) << gps.x, gps.y;
            R(current_row, current_row) = r_gains_.gps.position;
            R(current_row + 1, current_row + 1) = r_gains_.gps.position;

            current_row += GPS_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adding GPS (Pos. XY) measurement.");
        }

        // --- STEP 4: Perform safety checks before the update ---
        if (z.hasNaN())
        {
            RCLCPP_WARN_STREAM(logger_, "Combined measurements contain NaN! Correction skipped.");
            return;
        }

        // Covariance flooring: Ensure no variance on the diagonal of R is zero or negative.
        // This is a crucial protection against filter divergence.
        double min_variance = 1e-6;
        for (int i = 0; i < total_measurements; ++i)
        {
            if (R(i, i) < min_variance)
                R(i, i) = min_variance;
        }

        // --- STEP 5: EKF Update Equations ---
        // Measurement model prediction h(x). Since we measure states directly, it's a simple matrix multiplication.
        Eigen::VectorXd z_pred = H * x_;

        // Innovation (or residual): y = z - h(x)
        Eigen::VectorXd y = z - z_pred;

        if (has_q_measurement)
        {
            // Special handling for quaternion innovation. Simple subtraction is incorrect for rotations.
            // 1a. Build quaternions from the measurement vector (z) and state vector (x_).
            Eigen::Quaterniond q_z(z(State::QW), z(State::QX), z(State::QY), z(State::QZ));

            Eigen::Quaterniond q_z_pred(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));

            // 1b. Normalize to ensure they are pure rotations.
            q_z.normalize();

            q_z_pred.normalize();

            // 1c. Ensure the shortest path for interpolation/error calculation.
            if (q_z.coeffs().dot(q_z_pred.coeffs()) < 0.0)
            {
                q_z.coeffs() *= -1.0;
            }

            // 1d. Calculate the rotation error: q_error = q_measured * q_predicted_inverse
            Eigen::Quaterniond q_error = q_z * q_z_pred.inverse();

            // 1e. Convert the error quaternion to a 3D rotation vector (axis * angle).
            Eigen::AngleAxisd error_angle_axis(q_error);

            Eigen::Vector3d attitude_error_3d = error_angle_axis.axis() * error_angle_axis.angle();

            // 1f. Insert the correct 3D attitude error into the innovation vector 'y'.
            y.segment<3>(State::QX) = attitude_error_3d;

            y(State::QW) = 0; // The error of the W component is zero by definition in the 3D error space.
        }

        // Innovation Covariance: S = H * P * H^T + R
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;

        // Kalman Gain: K = P * H^T * S^-1
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // --- STEP 6: Update State and Covariance ---
        // Calculate the full correction vector.
        Eigen::VectorXd correction = K * y;

        // Update the quaternion (multiplicatively).
        Eigen::Quaterniond q_old(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
        Eigen::Vector3d attitude_correction_3d = correction.segment<3>(State::QX);

        // Convert the 3D correction vector back to a small correction quaternion.
        Eigen::Quaterniond q_correction;
        double angle = attitude_correction_3d.norm();
        if (angle > 0.0)
        {
            Eigen::Vector3d axis = attitude_correction_3d.normalized();
            q_correction = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }
        else
        {
            q_correction.setIdentity();
        }
        // Apply the correction and re-normalize.
        Eigen::Quaterniond q_new = q_correction * q_old;
        x_(State::QW) = q_new.normalized().w();
        x_(State::QX) = q_new.normalized().x();
        x_(State::QY) = q_new.normalized().y();
        x_(State::QZ) = q_new.normalized().z();

        // Update the linear states (additively).
        x_.segment<3>(State::PX) += correction.segment<3>(State::PX);
        x_.segment<3>(State::VX) += correction.segment<3>(State::VX);
        x_.segment<3>(State::WX) += correction.segment<3>(State::WX);

        // Update the covariance: P = (I - K * H) * P
        Eigen::Matrix<double, STATES, STATES> I;
        I.setIdentity();
        P_ = (I - K * H) * P_;

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "State (x) AFTER:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Position (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quaternion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Lin. Velocity (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Ang. Velocity (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Innovation (norm y): " << y.norm());
            RCLCPP_DEBUG_STREAM(logger_, "Uncertainty (trace P) AFTER: " << P_.trace());

            RCLCPP_DEBUG_STREAM(logger_, "--- Confiança do Filtro (Norma do Ganho de Kalman K) ---");
            int print_row = 0;

            if (measurements.px4_odometry)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ PX4 Odom: " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.openvins)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ OpenVINS: " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.fast_lio)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ FastLIO:  " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.imu)
            {
                double norm = K.block(0, print_row, STATES, IMU_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ IMU (W):  " << norm);
                print_row += IMU_MEASUREMENTS;
            }
            if (measurements.gps)
            {
                double norm = K.block(0, print_row, STATES, GPS_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  └─ GPS (XY): " << norm);
                print_row += GPS_MEASUREMENTS;
            }
        }
    }

    /**
     * @brief Calculates a custom attitude error metric from two quaternions.
     * @param q The measured quaternion.
     * @param q_ref The reference (estimated) quaternion.
     * @return A 3D error vector calculated via a specific formula.
     */
    Eigen::Vector3d StateEstimator::calculate_custom_attitude_error(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_ref)
    {
        // Calculate the error quaternion.
        Eigen::Quaterniond q_error = q * q_ref.inverse();

        double w_e = q_error.w();
        double x_e = q_error.x();
        double y_e = q_error.y();
        double z_e = q_error.z();

        // Calculate the error metric.
        // Denominator with a small epsilon to avoid division by zero.
        double q_att_denom = std::sqrt(w_e * w_e + z_e * z_e + 1e-3);

        // Numerator.
        Eigen::Vector3d q_att_num;
        q_att_num << w_e * x_e - y_e * z_e,
            w_e * y_e + x_e * z_e,
            z_e;

        // Final error vector.
        Eigen::Vector3d q_att = q_att_num / q_att_denom;

        return q_att;
    }

} // namespace laser_uav_estimators
