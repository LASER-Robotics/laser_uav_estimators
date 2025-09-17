/**
 * @file state_estimator.hpp
 * @brief Definition of the StateEstimator class, an Extended Kalman Filter implementation
 * for drone state estimation.
 * @author Wagner Dantas Garcia / Laser UAV Team
 * @date September 10, 2025
 */
#ifndef LASER_UAV_ESTIMATOR_STATE_ESTIMATOR_HPP
#define LASER_UAV_ESTIMATOR_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <optional>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// Include ROS message types for measurements
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>
#include <laser_uav_lib/kalman_filter/kalman_filter.hpp> // Include the base interface
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <type_traits>
#include <sstream>
#include <cmath>

namespace laser_uav_estimator
{
    /**
     * @brief Namespace that defines the state vector indices for easy access
     * and code readability.
     */
    namespace State
    {
        enum
        {
            PX = 0,  ///< Position in the X-axis of the inertial frame [m].
            PY = 1,  ///< Position in the Y-axis of the inertial frame [m].
            PZ = 2,  ///< Position in the Z-axis of the inertial frame [m].
            QW = 3,  ///< W component (real) of the orientation quaternion.
            QX = 4,  ///< X component (i) of the orientation quaternion.
            QY = 5,  ///< Y component (j) of the orientation quaternion.
            QZ = 6,  ///< Z component (k) of the orientation quaternion.
            VX = 7,  ///< Linear velocity in the X-axis of the BODY frame [m/s].
            VY = 8,  ///< Linear velocity in the Y-axis of the BODY frame [m/s].
            VZ = 9,  ///< Linear velocity in the Z-axis of the BODY frame [m/s].
            WX = 10, ///< Angular velocity in the X-axis of the BODY frame (Roll rate) [rad/s].
            WY = 11, ///< Angular velocity in the Y-axis of the BODY frame (Pitch rate) [rad/s].
            WZ = 12  ///< Angular velocity in the Z-axis of the BODY frame (Yaw rate) [rad/s].
        };
    }

    // --- Dimension Definitions ---
    constexpr int STATES = 13;       ///< Total number of states in the state vector.
    constexpr int INPUTS = 4;        ///< Number of control inputs (thrust of each motor).
    constexpr int MEASUREMENTS = 13; ///< Maximum number of measurements (used in odometry).

    /**
     * @brief Structure to group all available measurements in a single time step.
     * The use of `std::optional` allows any sensor measurement to be absent in the correction step,
     * making the filter flexible to sensor failures or absences.
     */
    struct MeasurementPackage
    {
        std::optional<nav_msgs::msg::Odometry> openvins;     ///< Odometry measurement from OpenVINS.
        std::optional<nav_msgs::msg::Odometry> fast_lio;     ///< Odometry measurement from Fast-LIO.
        std::optional<nav_msgs::msg::Odometry> px4_odometry; ///< Odometry measurement from PX4.
        std::optional<sensor_msgs::msg::Imu> imu;            ///< IMU measurement (acceleration and angular velocity).
        std::optional<geometry_msgs::msg::Point> gps;        ///< GPS measurement (position only).
    };

    /**
     * @brief Structure to configure the PROCESS (dynamic model) noise gains.
     * These values are used to build the process covariance matrix Q.
     */
    struct ProcessNoiseGains
    {
        double position = 0.01;
        double orientation = 0.01;
        double linear_velocity = 0.1;
        double angular_velocity = 0.1;
    };

    /**
     * @brief Structure to configure the MEASUREMENT (sensor) noise gains.
     * These values are used to build the measurement covariance matrix R for each sensor.
     */
    struct MeasurementNoiseGains
    {
        ProcessNoiseGains px4_odometry;
        ProcessNoiseGains openvins;
        ProcessNoiseGains fast_lio;
        ProcessNoiseGains imu;
        ProcessNoiseGains gps;
    };

    /**
     * @brief Extended Kalman Filter (EKF) implementation for a quadrotor.
     *
     * This class inherits from the base `EKF` class and specializes it to estimate
     * a drone's state (position, orientation, and velocities) by fusing data from multiple sensors.
     *
     * @details
     * A key feature of this implementation is the use of automatic differentiation
     * (via the `autodiff` library) for calculating the Jacobian matrices. This
     * simplifies the state transition model implementation and drastically reduces
     * the likelihood of mathematical errors. The filter is designed to be robust against
     * sensor failures by using `std::optional` to handle missing measurements.
     */
    class StateEstimator : public laser_uav_lib::EKF<STATES, INPUTS, MEASUREMENTS>
    {
    public:
        /**
         * @brief Constructor for the StateEstimator class.
         *
         * @param mass The drone's mass in kg.
         * @param motor_positions The (x,y) positions of the 4 motors in the body frame.
         * @param thrust_coeff The thrust coefficient of the motors.
         * @param torque_coeff The torque coefficient of the motors.
         * @param inertia The 3x3 inertia matrix of the drone.
         * @param verbosity The logging level for debugging ("SILENT", "INFO", "DEBUG").
         */
        StateEstimator(const double &mass, const Eigen::Matrix<double, 4, 2> &motor_positions, const double &thrust_coeff, const double &torque_coeff, const Eigen::Matrix3d &inertia, const std::string &verbosity = "INFO");

        /**
         * @brief Default destructor.
         */
        ~StateEstimator() = default;

        /**
         * @brief Executes the EKF prediction step.
         *
         * Projects the current state and covariance to the next time step,
         * based on the drone's motion model and the control inputs.
         *
         * @param u The control vector (forces of the 4 motors).
         * @param dt The time interval (delta t) since the last prediction, in seconds.
         */
        void predict(const Eigen::Matrix<double, INPUTS, 1> &u, double dt) override;

        /**
         * @brief Executes the EKF correction step using the available measurements.
         *
         * This method acts as a dispatcher, calling the specific correction methods
         * for each sensor present in the `MeasurementPackage`.
         *
         * @param measurements A package containing the available sensor data for this time step.
         */
        void correct(const MeasurementPackage &measurements);

        /**
         * @brief Overload of the base class's correction method.
         * @note This implementation is not used directly. The correction is handled by the
         * method that accepts a `MeasurementPackage`. This function exists to satisfy the
         * `EKF` base class interface.
         * @param z Generic measurement vector (not used).
         */
        void correct(const Eigen::Matrix<double, MEASUREMENTS, 1> &z) override
        {
            // This implementation can be left empty or throw an error,
            // since the correction is handled by the method accepting MeasurementPackage.
            (void)z; // Avoid unused variable warning.
        }

        /**
         * @brief Resets the filter's state and covariance to their initial values.
         * Useful for re-initializing the estimate after a failure or when the drone is in a known state.
         */
        void reset();

        // --- Getters & Setters ---
        const Eigen::Matrix<double, STATES, 1> &get_state() const override { return x_; }
        const Eigen::Matrix<double, STATES, STATES> &get_covariance() const override { return P_; }

        /**
         * @brief Sets the process noise covariance matrix (Q).
         * @param Q The new process noise covariance matrix.
         */
        void set_process_noise(const Eigen::Matrix<double, STATES, STATES> &Q) { Q_ = Q; }

        /**
         * @brief Sets the verbosity level of the logs.
         * @param verbosity The verbosity string ("ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT").
         */
        void set_verbosity(const std::string &verbosity);

        /**
         * @brief Gets the current verbosity level.
         * @return std::string The verbosity level.
         */
        std::string get_verbosity() const;

        /**
         * @brief Sets the process noise gains and updates the Q matrix.
         * @param gains The struct with the new process noise gains.
         */
        void set_process_noise_gains(const ProcessNoiseGains &gains);

        /**
         * @brief Sets the measurement noise gains for each sensor.
         * @param gains The struct with the new measurement noise gains.
         */
        void set_measurement_noise_gains(const MeasurementNoiseGains &gains);

        /**
         * @brief Calculates a custom attitude error metric from two quaternions.
         * @details This function calculates the error between the measured and estimated orientation.
         * The specific formula can be adapted as needed by the control system.
         * @param q The measured quaternion.
         * @param q_ref The reference quaternion (estimated by the filter).
         * @return A 3D error vector representing the attitude difference.
         */
        Eigen::Vector3d calculate_custom_attitude_error(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_ref);

    private:
        // --- INTERNAL CORRECTION METHODS ---

        /** @brief Corrects the state using a generic odometry measurement. */
        void correct_odometry(const nav_msgs::msg::Odometry &odom, const ProcessNoiseGains &gains);

        /** @brief Corrects the state using an IMU measurement. */
        void correct_imu(const sensor_msgs::msg::Imu &imu);

        /** @brief Corrects the state using a GPS measurement. */
        void correct_gps(const geometry_msgs::msg::Point &gps);

        // --- MATHEMATICAL MODEL AND JACOBIANS ---

        /**
         * @brief The drone's non-linear state transition model.
         *
         * Implements the differential equations describing the physics of the drone's motion.
         * It is a template method so it can be used with `autodiff::real` for automatic differentiation.
         *
         * @tparam T The scalar type (can be `double` or `autodiff::real`).
         * @param x The current state vector.
         * @param u The applied control vector (motor forces).
         * @return The vector of state derivatives (x_dot).
         */
        template <typename T>
        Eigen::Matrix<T, STATES, 1> state_transition_model(const Eigen::Matrix<T, STATES, 1> &x, const Eigen::Matrix<T, INPUTS, 1> &u) const;

        /**
         * @brief Calculates the Jacobian matrix F (derivative of the transition model with respect to the state).
         * It uses automatic differentiation to avoid manual calculation.
         *
         * @param x The state vector at which to evaluate the Jacobian.
         * @param u The control vector at which to evaluate the Jacobian.
         */
        void calculate_jacobian_F(const Eigen::Matrix<double, STATES, 1> &x, const Eigen::Matrix<double, INPUTS, 1> &u);

        /**
         * @brief Fuses multiple odometry sources using inverse-variance weighting.
         *
         * @param measurements The measurement package containing the odometry sources.
         * @return An `std::optional` containing a pair of:
         * - The fused measurement vector (z).
         * - Its combined covariance matrix (R).
         * Returns an empty optional if no odometry sources are available.
         */
        std::optional<std::pair<Eigen::Matrix<double, 13, 1>, Eigen::Matrix<double, 13, 13>>> fuse_odometry(const MeasurementPackage &measurements);

        /**
         * @brief Updates the process noise covariance matrix (Q) based on the defined gains.
         */
        void update_Q_matrix();

        // --- FILTER MEMBERS ---
        double dt_; ///< Stores the last time interval (dt) for use in the Jacobian [s].

        // --- PHYSICAL PARAMETERS ---
        double mass_;                                 ///< Drone mass [kg].
        Eigen::Matrix<double, 4, 2> motor_positions_; ///< (x,y) positions of the motors in the body frame [m].
        double arm_length_;                           ///< Drone arm length (center to motor) [m].
        double thrust_coefficient_;                   ///< Thrust coefficient [N/rpm^2].
        double torque_coefficient_;                   ///< Torque coefficient [Nm/rpm^2].
        Eigen::Matrix3d inertia_tensor_;              ///< Inertia tensor [kg*m^2].
        Eigen::Matrix3d inertia_tensor_inv_;          ///< Inverse of the inertia tensor (pre-calculated).
        static constexpr double GRAVITY = 9.80665;    ///< Acceleration of gravity [m/s^2].

        // --- LOGGING AND DEBUGGING ---
        rclcpp::Logger logger_; ///< ROS2 logger for displaying messages.
        std::string verbosity_; ///< Current log level as a string.
        bool is_debug_;         ///< Boolean flag to optimize real-time log checks.

        ProcessNoiseGains q_gains_;     ///< Gains for the process noise matrix (Q).
        MeasurementNoiseGains r_gains_; ///< Gains for the measurement noise matrix (R).
    };
} // namespace laser_uav_estimator

#endif // LASER_UAV_ESTIMATOR_state_estimator_HPP