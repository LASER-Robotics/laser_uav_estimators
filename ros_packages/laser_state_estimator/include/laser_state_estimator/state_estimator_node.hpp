/**
 * @file state_estimator_node.hpp
 * @brief Implementation of the StateEstimator class methods.
 * @author Wagner Dantas Garcia / Laser UAV Team
 * @date September 10, 2025
 */

#ifndef LASER_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define LASER_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

/* includes //{ */
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/msg/state_estimator_diagnostics.hpp>
#include <laser_msgs/msg/sensor_status.hpp>
#include <laser_msgs/srv/set_string.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <chrono>

#include <laser_uav_lib/kalman_filter/drone_ekf/drone_ekf.hpp>
/*//}*/

/* define //{*/
/**
 * @brief Type alias for the return type of lifecycle node callbacks.
 */
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/*//}*/

namespace laser_state_estimator
{
  /**
   * @brief A thread-safe buffer for storing and managing time-stamped sensor data.
   * @tparam MsgT The ROS message type to be stored in the buffer.
   */
  template <typename MsgT>
  struct SensorDataBuffer
  {
    /**
     * @brief Construct a new Sensor Data Buffer object, initializing durations to zero.
     */
    SensorDataBuffer() : tolerance(0, 0), timeout(0, 0) {}

    std::map<rclcpp::Time, typename MsgT::SharedPtr> buffer; ///< The buffer storing messages, keyed by their timestamp.
    std::mutex mtx;                                          ///< Mutex for thread-safe access to the buffer.
    rclcpp::Duration tolerance;                              ///< Time tolerance for message synchronization.
    rclcpp::Duration timeout;                                ///< Maximum age for a message before it is considered stale.
    bool is_active{false};                                   ///< Flag indicating if this sensor source is currently active.
  };

  /**
   * @brief A ROS 2 lifecycle node for state estimation of a UAV.
   *
   * This class implements an Extended Kalman Filter (EKF) to fuse data from various sensors
   * like IMU and multiple odometry sources (e.g., PX4, OpenVINS, FastLIO) to provide a
   * robust and accurate state estimate (position, orientation, velocity) of the UAV.
   */
  class StateEstimator : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    /**
     * @brief Construct a new State Estimator object.
     * @param options Node options for initialization.
     */
    explicit StateEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the State Estimator object.
     */
    ~StateEstimator() override;

  private:
    /* CONFIG //{ */
    // Lifecycle Callbacks
    /**
     * @brief Callback for the "configuring" transition.
     *
     * Sets up parameters, publishers, subscribers, timers, and services.
     * @param state The previous state of the node.
     * @return CallbackReturn::SUCCESS on success, CallbackReturn::FAILURE on error.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Callback for the "activating" transition.
     *
     * Activates publishers and timers, and starts the estimation process.
     * @param state The previous state of the node.
     * @return CallbackReturn::SUCCESS on success, CallbackReturn::FAILURE on error.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Callback for the "deactivating" transition.
     *
     * Deactivates publishers and timers, pausing the estimation process.
     * @param state The previous state of the node.
     * @return CallbackReturn::SUCCESS on success, CallbackReturn::FAILURE on error.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Callback for the "cleaning up" transition.
     *
     * Releases resources allocated during the "configuring" state.
     * @param state The previous state of the node.
     * @return CallbackReturn::SUCCESS on success, CallbackReturn::FAILURE on error.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Callback for the "shutting down" transition.
     *
     * Performs final cleanup before the node is destroyed.
     * @param state The previous state of the node.
     * @return CallbackReturn::SUCCESS on success, CallbackReturn::FAILURE on error.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    // Setup Functions
    /**
     * @brief Loads and validates parameters from the ROS parameter server.
     */
    void getParameters();

    /**
     * @brief Configures all publishers and subscribers for the node.
     */
    void configPubSub();

    /**
     * @brief Configures all timers for periodic tasks.
     */
    void configTimers();

    /**
     * @brief Configures all services offered by the node.
     */
    void configServices();
    /*//}*/

    /* CALLBACKS //{ */
    /**
     * @brief Callback for odometry messages from PX4.
     * @param msg The received odometry message.
     */
    void odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for odometry messages from OpenVINS.
     * @param msg The received odometry message.
     */
    void odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for odometry messages from Fast-LIO.
     * @param msg The received odometry message.
     */
    void odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for IMU messages.
     * @param msg The received IMU message.
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Callback for control diagnostics messages, containing control inputs.
     * @param msg The received control diagnostics message.
     */
    void controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg);

    /**
     * @brief Main timer callback for the EKF prediction and update steps.
     *
     * This function runs at a fixed frequency, synchronizes sensor data,
     * performs the EKF prediction, applies corrections with available measurements,
     * and publishes the resulting state estimate.
     */
    void timerCallback();

    /**
     * @brief Timer callback to periodically check the status of subscribers.
     */
    void checkSubscribersCallback();

    /**
     * @brief Timer callback to publish diagnostic information about the estimator.
     */
    void diagnosticsTimerCallback();
    /*//}*/

    /**
     * @brief Callback for the service that sets the active odometry source.
     * @param request The service request, containing the name of the desired source.
     * @param response The service response, indicating success or failure.
     */
    void setOdometryCallback(
        const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
        std::shared_ptr<laser_msgs::srv::SetString::Response> response);

    rclcpp::Service<laser_msgs::srv::SetString>::SharedPtr set_odometry_service_; ///< Server for the odometry selection service.

    /* FUNCTIONS //{ */
    /**
     * @brief Initializes and configures the Extended Kalman Filter with loaded parameters.
     */
    void setupEKF();

    /**
     * @brief Publishes the current EKF state as an Odometry message.
     * @param pub The lifecycle publisher to use for publishing.
     */
    void publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub);

    /**
     * @brief Retrieves a message from a sensor buffer that is time-synchronized with a reference time.
     *
     * Finds the message in the buffer closest to the reference time, within a specified tolerance.
     * @tparam MsgT The message type.
     * @param ref_time The reference timestamp for synchronization.
     * @param sensor_data The sensor data buffer to search in.
     * @param sensor_name The name of the sensor for logging purposes.
     * @return An optional containing the synchronized message if found, otherwise std::nullopt.
     */
    template <typename MsgT>
    std::optional<MsgT> getSynchronizedMessage(const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name);

    /**
     * @brief Removes old messages from a sensor buffer.
     *
     * This prevents the buffer from growing indefinitely by discarding messages
     * that are older than the defined timeout.
     * @tparam MsgT The message type.
     * @param now The current time.
     * @param sensor_data The sensor data buffer to prune.
     */
    template <typename MsgT>
    void pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data);
    /*//}*/

    /* EKF //{ */
    std::unique_ptr<laser_uav_lib::DroneEKF> ekf_; ///< Unique pointer to the Drone EKF instance.
    /*//}*/

    /* ROS COMMUNICATIONS //{ */
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;                           ///< Publisher for the final estimated odometry.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr predict_pub_;                        ///< Publisher for the odometry prediction step (for debugging).
    rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::StateEstimatorDiagnostics>::SharedPtr diagnostics_pub_; ///< Publisher for estimator diagnostics.

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_px4_sub_;           ///< Subscriber for PX4 odometry.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_openvins_sub_;      ///< Subscriber for OpenVINS odometry.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_fast_lio_sub_;      ///< Subscriber for Fast-LIO odometry.
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                      ///< Subscriber for IMU data.
    rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::SharedPtr control_sub_; ///< Subscriber for control input data.
    rclcpp::TimerBase::SharedPtr timer_;                                                  ///< Main EKF update timer.
    rclcpp::TimerBase::SharedPtr check_subscribers_timer_;                                ///< Timer to check subscriber status.
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;                                      ///< Timer to publish diagnostics.
    /*//}*/

    /* STATE & THREAD-SAFETY //{ */
    std::mutex mtx_;                                            ///< Main mutex for protecting shared data access.
    rclcpp::Time last_update_time_;                             ///< Timestamp of the last EKF update.
    rclcpp::Time last_px4_odom_time_;                           ///< Timestamp of the last received PX4 odometry message.
    rclcpp::Time last_openvins_odom_time_;                      ///< Timestamp of the last received OpenVINS odometry message.
    rclcpp::Time last_fast_lio_odom_time_;                      ///< Timestamp of the last received Fast-LIO odometry message.
    rclcpp::Time last_imu_time_;                                ///< Timestamp of the last received IMU message.
    rclcpp::Time last_control_input_time_;                      ///< Timestamp of the last received control input message.
    std::chrono::steady_clock::time_point last_cpp_time_point_; ///< Last C++ steady clock time point for calculating dt.

    SensorDataBuffer<nav_msgs::msg::Odometry> px4_odom_data_;               ///< Data buffer for PX4 odometry.
    SensorDataBuffer<nav_msgs::msg::Odometry> openvins_odom_data_;          ///< Data buffer for OpenVINS odometry.
    SensorDataBuffer<nav_msgs::msg::Odometry> fast_lio_odom_data_;          ///< Data buffer for Fast-LIO odometry.
    SensorDataBuffer<sensor_msgs::msg::Imu> imu_data_;                      ///< Data buffer for IMU messages.
    SensorDataBuffer<laser_msgs::msg::UavControlDiagnostics> control_data_; ///< Data buffer for control input messages.

    nav_msgs::msg::Odometry::SharedPtr last_odometry_px4_msg_;             ///< Pointer to the last received PX4 odometry message.
    nav_msgs::msg::Odometry::SharedPtr last_odometry_openvins_msg_;        ///< Pointer to the last received OpenVINS odometry message.
    nav_msgs::msg::Odometry::SharedPtr last_odometry_fast_lio_msg_;        ///< Pointer to the last received Fast-LIO odometry message.
    sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;                        ///< Pointer to the last received IMU message.
    laser_msgs::msg::UavControlDiagnostics::SharedPtr last_control_input_; ///< Pointer to the last received control input message.

    bool is_active_{false};            ///< Flag indicating if the node is in the active state.
    bool is_control_input_{false};     ///< Flag indicating if control inputs are being received.
    bool is_initialized_{false};       ///< Flag indicating if the EKF has been initialized.
    bool is_first_control_msg{false};  ///< Flag to handle the first control message specially.
    bool enable_px4_odom_{false};      ///< Flag to enable/disable the PX4 odometry source.
    bool enable_openvins_odom_{false}; ///< Flag to enable/disable the OpenVINS odometry source.
    bool enable_fast_lio_odom_{false}; ///< Flag to enable/disable the Fast-LIO odometry source.

    std::string current_active_odometry_name_{"NONE"};  ///< Name of the currently active odometry source.
    std::vector<std::string> odometry_source_names_;    ///< List of available odometry source names.
    double odometry_switch_distance_threshold_;         ///< Position difference threshold to switch odometry source.
    double odometry_switch_angle_threshold_;            ///< Angle difference threshold to switch odometry source.
    double odometry_switch_velocity_linear_threshold_;  ///< Linear velocity difference threshold to switch odometry source.
    double odometry_switch_velocity_angular_threshold_; ///< Angular velocity difference threshold to switch odometry source.
    /*//}*/

    /* PARAMETERS //{ */
    double frequency_;                      ///< Main loop frequency in Hz.
    double sensor_timeout_;                 ///< General timeout for sensors in seconds.
    std::string ekf_verbosity_;             ///< Verbosity level for the EKF library.
    double mass_;                           ///< Mass of the UAV in kg.
    double arm_length_;                     ///< Arm length of the UAV in meters.
    double thrust_coefficient_;             ///< Motor thrust coefficient.
    double torque_coefficient_;             ///< Motor torque coefficient.
    std::vector<double> inertia_vec_;       ///< Inertia matrix elements [Ixx, Iyy, Izz].
    std::vector<double> motor_positions_;   ///< Motor positions relative to the center of mass.
    std::vector<double> process_noise_vec_; ///< Vector of process noise values for the EKF.

    laser_uav_lib::ProcessNoiseGains process_noise_gains_;         ///< Struct holding process noise gains for the EKF.
    laser_uav_lib::MeasurementNoiseGains measurement_noise_gains_; ///< Struct holding measurement noise gains for the EKF.

    // Sensor-specific synchronization parameters (in seconds)
    double px4_odom_tolerance_;      ///< Synchronization time tolerance for PX4 odometry.
    double px4_odom_timeout_;        ///< Data timeout for PX4 odometry.
    double openvins_odom_tolerance_; ///< Synchronization time tolerance for OpenVINS odometry.
    double openvins_odom_timeout_;   ///< Data timeout for OpenVINS odometry.
    double fast_lio_odom_tolerance_; ///< Synchronization time tolerance for Fast-LIO odometry.
    double fast_lio_odom_timeout_;   ///< Data timeout for Fast-LIO odometry.
    double imu_tolerance_;           ///< Synchronization time tolerance for IMU data.
    double imu_timeout_;             ///< Data timeout for IMU data.
    double control_tolerance_;       ///< Synchronization time tolerance for control inputs.
    double control_timeout_;         ///< Data timeout for control inputs.

    /*//}*/
  };
} // namespace laser_state_estimator

#endif // LASER_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_