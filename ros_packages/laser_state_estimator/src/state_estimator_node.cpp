/**
 * @file state_estimator_node.cpp
 * @brief Implements the StateEstimator lifecycle node for robust UAV state estimation.
 * This node utilizes an Extended Kalman Filter (EKF) to fuse measurements from an
 * IMU and multiple, dynamically selectable odometry sources (e.g., PX4, VIO, LIO).
 * It incorporates the UAV's dynamic model by using control inputs for the prediction
 * step. Key features include thread-safe data buffering, time synchronization of
 * sensor inputs, a ROS service for on-the-fly odometry source switching, and the
 * publication of detailed diagnostics.
 * @author Wagner Dantas Garcia / Laser UAV Team
 * @date September 10, 2025
 */
#include <laser_state_estimator/state_estimator_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_state_estimator
{
  /**
   * @brief Construct a new State Estimator:: State Estimator object.
   *
   * Initializes the lifecycle node and declares all necessary parameters for the EKF,
   * drone model, sensor configurations, and noise gains.
   *
   * @param options The node options for rclcpp_lifecycle::LifecycleNode.
   */
  StateEstimator::StateEstimator(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("state_estimator", options)
  {
    RCLCPP_INFO(get_logger(), "Creating StateEstimator...");

    // --- Parameter Declarations ---
    // General node parameters
    declare_parameter("frequency", 100.0);
    declare_parameter("initial_odometry_source", "px4_api_odom");
    declare_parameter("odometry_source_names", std::vector<std::string>{});
    declare_parameter("odometry_switch_distance_threshold", 0.25);
    declare_parameter("odometry_switch_angle_threshold", 0.5);
    declare_parameter("odometry_switch_velocity_linear_threshold", 0.5);
    declare_parameter("odometry_switch_velocity_angular_threshold", 1.0);
    declare_parameter("sensor_timeout", 0.5);
    declare_parameter("ekf_verbosity", "INFO");

    // Drone physical parameters
    declare_parameter("drone_params.mass", 1.0);
    declare_parameter("drone_params.arm_length", 0.25);
    declare_parameter("drone_params.thrust_coefficient", 1.0);
    declare_parameter("drone_params.torque_coefficient", 0.05);
    declare_parameter("drone_params.inertia", std::vector<double>{0.01, 0.01, 0.01});
    declare_parameter("drone_params.motor_positions", std::vector<double>{0.1, -0.1, -0.1, 0.1, 0.1, 0.1, -0.1, -0.1});

    // EKF process noise gains
    declare_parameter("process_noise_gains.position", 0.01);
    declare_parameter("process_noise_gains.orientation", 0.01);
    declare_parameter("process_noise_gains.linear_velocity", 0.1);
    declare_parameter("process_noise_gains.angular_velocity", 0.1);

    // Measurement noise gains for each sensor
    declare_parameter("measurement_noise_gains.px4_odometry.position", 1.0);
    declare_parameter("measurement_noise_gains.px4_odometry.orientation", 1.0);
    declare_parameter("measurement_noise_gains.px4_odometry.linear_velocity", 1.0);
    declare_parameter("measurement_noise_gains.px4_odometry.angular_velocity", 1.0);
    declare_parameter("measurement_noise_gains.openvins.position", 1.0);
    declare_parameter("measurement_noise_gains.openvins.orientation", 1.0);
    declare_parameter("measurement_noise_gains.openvins.linear_velocity", 1.0);
    declare_parameter("measurement_noise_gains.openvins.angular_velocity", 1.0);
    declare_parameter("measurement_noise_gains.fast_lio.position", 1.0);
    declare_parameter("measurement_noise_gains.fast_lio.orientation", 1.0);
    declare_parameter("measurement_noise_gains.fast_lio.linear_velocity", 1.0);
    declare_parameter("measurement_noise_gains.fast_lio.angular_velocity", 1.0);
    declare_parameter("measurement_noise_gains.imu.angular_velocity", 1.0);
    declare_parameter("measurement_noise_gains.gps.position", 1.0);

    // Synchronization parameters (tolerance and timeout) for each sensor
    declare_parameter("px4_odom_tolerance", 0.1);
    declare_parameter("px4_odom_timeout", 0.5);
    declare_parameter("openvins_odom_tolerance", 0.1);
    declare_parameter("openvins_odom_timeout", 0.5);
    declare_parameter("fast_lio_odom_tolerance", 0.1);
    declare_parameter("fast_lio_odom_timeout", 0.5);
    declare_parameter("imu_tolerance", 0.1);
    declare_parameter("imu_timeout", 0.5);
    declare_parameter("control_tolerance", 0.1);
    declare_parameter("control_timeout", 0.5);

    RCLCPP_INFO(get_logger(), "StateEstimator node initialized.");
  }

  /**
   * @brief Destroy the State Estimator:: State Estimator object.
   */
  StateEstimator::~StateEstimator() {}

  /**
   * @brief Lifecycle callback for the 'configuring' state transition.
   */
  CallbackReturn StateEstimator::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring StateEstimator...");

    // Execute the configuration sequence
    getParameters();  // Load parameters from the ROS parameter server
    configPubSub();   // Configure publishers and subscribers
    configTimers();   // Configure the main loop and diagnostics timers
    configServices(); // Configure ROS services
    setupEKF();       // Initialize the EKF with the loaded parameters

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Lifecycle callback for the 'activating' state transition.
   */
  CallbackReturn StateEstimator::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating StateEstimator...");
    // Activate publishers so they can start publishing messages
    odom_pub_->on_activate();
    predict_pub_->on_activate();
    diagnostics_pub_->on_activate();

    // Set the node's main flag to active
    is_active_ = true;
    // Reset (start) the timers
    timer_->reset();
    diagnostics_timer_->reset();
    // Reset the EKF state for a fresh start
    ekf_->reset();
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Lifecycle callback for the 'deactivating' state transition.
   */
  CallbackReturn StateEstimator::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating StateEstimator...");
    // Set the node's main flag to inactive
    is_active_ = false;
    // Cancel (stop) the timers
    timer_->cancel();
    diagnostics_timer_->cancel();
    // Deactivate the publishers
    odom_pub_->on_deactivate();
    predict_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Lifecycle callback for the 'cleaning up' state transition.
   */
  CallbackReturn StateEstimator::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up StateEstimator...");
    // Release memory and resources by resetting the smart pointers
    ekf_.reset();
    odom_pub_.reset();
    predict_pub_.reset();
    diagnostics_pub_.reset();
    odometry_px4_sub_.reset();
    odometry_fast_lio_sub_.reset();
    odometry_openvins_sub_.reset();
    imu_sub_.reset();
    control_sub_.reset();
    timer_.reset();
    diagnostics_timer_.reset();

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Lifecycle callback for the 'shutting down' state transition.
   */
  CallbackReturn StateEstimator::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down StateEstimator...");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Retrieves parameters from the ROS parameter server.
   */
  void StateEstimator::getParameters()
  {
    RCLCPP_INFO(get_logger(), "Loading parameters...");
    // Get each parameter and store it in the corresponding member variable
    get_parameter("frequency", frequency_);
    get_parameter("initial_odometry_source", current_active_odometry_name_);
    get_parameter("odometry_source_names", odometry_source_names_);
    get_parameter("odometry_switch_distance_threshold", odometry_switch_distance_threshold_);
    get_parameter("odometry_switch_angle_threshold", odometry_switch_angle_threshold_);
    get_parameter("odometry_switch_velocity_linear_threshold", odometry_switch_velocity_linear_threshold_);
    get_parameter("odometry_switch_velocity_angular_threshold", odometry_switch_velocity_angular_threshold_);
    get_parameter("sensor_timeout", sensor_timeout_);
    get_parameter("ekf_verbosity", ekf_verbosity_);
    get_parameter("drone_params.mass", mass_);
    get_parameter("drone_params.arm_length", arm_length_);
    get_parameter("drone_params.thrust_coefficient", thrust_coefficient_);
    get_parameter("drone_params.torque_coefficient", torque_coefficient_);
    get_parameter("drone_params.inertia", inertia_vec_);
    get_parameter("drone_params.motor_positions", motor_positions_);
    get_parameter("process_noise_gains.position", process_noise_gains_.position);
    get_parameter("process_noise_gains.orientation", process_noise_gains_.orientation);
    get_parameter("process_noise_gains.linear_velocity", process_noise_gains_.linear_velocity);
    get_parameter("process_noise_gains.angular_velocity", process_noise_gains_.angular_velocity);
    get_parameter("measurement_noise_gains.px4_odometry.position", measurement_noise_gains_.px4_odometry.position);
    get_parameter("measurement_noise_gains.px4_odometry.orientation", measurement_noise_gains_.px4_odometry.orientation);
    get_parameter("measurement_noise_gains.px4_odometry.linear_velocity", measurement_noise_gains_.px4_odometry.linear_velocity);
    get_parameter("measurement_noise_gains.px4_odometry.angular_velocity", measurement_noise_gains_.px4_odometry.angular_velocity);
    get_parameter("measurement_noise_gains.openvins.position", measurement_noise_gains_.openvins.position);
    get_parameter("measurement_noise_gains.openvins.orientation", measurement_noise_gains_.openvins.orientation);
    get_parameter("measurement_noise_gains.openvins.linear_velocity", measurement_noise_gains_.openvins.linear_velocity);
    get_parameter("measurement_noise_gains.openvins.angular_velocity", measurement_noise_gains_.openvins.angular_velocity);
    get_parameter("measurement_noise_gains.fast_lio.position", measurement_noise_gains_.fast_lio.position);
    get_parameter("measurement_noise_gains.fast_lio.orientation", measurement_noise_gains_.fast_lio.orientation);
    get_parameter("measurement_noise_gains.fast_lio.linear_velocity", measurement_noise_gains_.fast_lio.linear_velocity);
    get_parameter("measurement_noise_gains.fast_lio.angular_velocity", measurement_noise_gains_.fast_lio.angular_velocity);
    get_parameter("measurement_noise_gains.imu.angular_velocity", measurement_noise_gains_.imu.angular_velocity);
    get_parameter("measurement_noise_gains.gps.position", measurement_noise_gains_.gps.position);

    double tolerance, timeout;

    // Set the tolerance and timeout for each sensor buffer
    get_parameter("px4_odom_tolerance", tolerance);
    get_parameter("px4_odom_timeout", timeout);
    px4_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    px4_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("openvins_odom_tolerance", tolerance);
    get_parameter("openvins_odom_timeout", timeout);
    openvins_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    openvins_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("fast_lio_odom_tolerance", tolerance);
    get_parameter("fast_lio_odom_timeout", timeout);
    fast_lio_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    fast_lio_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("imu_tolerance", tolerance);
    get_parameter("imu_timeout", timeout);
    imu_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    imu_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("control_tolerance", tolerance);
    get_parameter("control_timeout", timeout);
    control_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    control_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    RCLCPP_INFO(get_logger(), "Parameters loaded.");
  }

  /**
   * @brief Configures all ROS publishers and subscribers.
   */
  void StateEstimator::configPubSub()
  {
    RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");
    // Create publisher for the final estimated odometry
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
    // Create publisher for the prediction-step odometry (for debugging)
    predict_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);
    // Create publisher for diagnostic messages
    diagnostics_pub_ = create_publisher<laser_msgs::msg::StateEstimatorDiagnostics>("~/diagnostics", 10);

    // Create subscribers for each sensor data source
    odometry_px4_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&StateEstimator::odometryPx4Callback, this, std::placeholders::_1));
    odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_fast_lio_in", 10, std::bind(&StateEstimator::odometryFastLioCallback, this, std::placeholders::_1));
    odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_openvins_in", 10, std::bind(&StateEstimator::odometryOpenVinsCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", 10, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
    control_sub_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>("control_in", 10, std::bind(&StateEstimator::controlCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
  }

  /**
   * @brief Configures all timers for periodic tasks.
   */
  void StateEstimator::configTimers()
  {
    RCLCPP_INFO(get_logger(), "Configuring timers...");
    // Main timer that runs the EKF loop at the defined frequency
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&StateEstimator::timerCallback, this));
    // Timer to publish diagnostic messages at a lower frequency (1/10th of main)
    diagnostics_timer_ = create_wall_timer(std::chrono::duration<double>(1 / (frequency_ / 10)), std::bind(&StateEstimator::diagnosticsTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Timers configured.");
  }

  /**
   * @brief Configures all ROS services.
   */
  void StateEstimator::configServices()
  {
    RCLCPP_INFO(get_logger(), "Configuring services... ");
    // Create the service that allows an external client to set the active odometry source
    set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
        "~/set_odometry",
        std::bind(&StateEstimator::setOdometryCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  /**
   * @brief Initializes and configures the Extended Kalman Filter.
   */
  void StateEstimator::setupEKF()
  {
    RCLCPP_INFO(get_logger(), "Configuring EKF...");
    // Create the 3x3 inertia matrix from the parameter vector
    Eigen::Matrix3d inertia = Eigen::Vector3d(inertia_vec_[0], inertia_vec_[1], inertia_vec_[2]).asDiagonal();

    // Validate that the motor positions vector has the correct size (x,y for 4 motors)
    if (motor_positions_.size() != 8)
    {
      RCLCPP_ERROR(get_logger(), "motor_positions must have exactly 8 elements (x,y for 4 motors). Current: %zu", motor_positions_.size());
      throw std::invalid_argument("Incorrect size for motor_positions");
    }

    // Populate the 4x2 motor positions matrix from the flat vector
    Eigen::Matrix<double, 4, 2> motor_positions;
    motor_positions(0, 0) = motor_positions_[0]; // Motor 0 - x
    motor_positions(0, 1) = motor_positions_[1]; // Motor 0 - y
    motor_positions(1, 0) = motor_positions_[2]; // Motor 1 - x
    motor_positions(1, 1) = motor_positions_[3]; // Motor 1 - y
    motor_positions(2, 0) = motor_positions_[4]; // Motor 2 - x
    motor_positions(2, 1) = motor_positions_[5]; // Motor 2 - y
    motor_positions(3, 0) = motor_positions_[6]; // Motor 3 - x
    motor_positions(3, 1) = motor_positions_[7]; // Motor 3 - y

    // Instantiate the DroneEKF object with the drone's physical parameters
    ekf_ = std::make_unique<laser_uav_lib::DroneEKF>(mass_, motor_positions, thrust_coefficient_, torque_coefficient_, inertia, ekf_verbosity_);

    RCLCPP_INFO(get_logger(), "Applying noise gains to EKF.");
    // Set the process and measurement noise gains in the EKF
    ekf_->set_process_noise_gains(process_noise_gains_);
    ekf_->set_measurement_noise_gains(measurement_noise_gains_);

    // Set the initial odometry source based on the parameter
    if (current_active_odometry_name_ == "px4_api_odom")
    {
      enable_px4_odom_ = true;
      enable_openvins_odom_ = false;
      enable_fast_lio_odom_ = false;
    }
    else if (current_active_odometry_name_ == "openvins_odom")
    {
      enable_px4_odom_ = false;
      enable_openvins_odom_ = true;
      enable_fast_lio_odom_ = false;
    }
    else if (current_active_odometry_name_ == "fast_lio_odom")
    {
      enable_px4_odom_ = false;
      enable_openvins_odom_ = false;
      enable_fast_lio_odom_ = true;
    }
    else
    {
      // If the initial source is invalid, use a default and warn the user
      RCLCPP_WARN(get_logger(), "Invalid initial odometry source. Using 'px4_api_odom' as default.");
      current_active_odometry_name_ = "px4_api_odom";
      enable_px4_odom_ = true;
      enable_openvins_odom_ = false;
      enable_fast_lio_odom_ = false;
    }

    RCLCPP_INFO(get_logger(), "EKF configured.");
  }

  /**
   * @brief Callback for incoming odometry messages from PX4.
   */
  void StateEstimator::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Debug log, showing the message frequency
    if (last_odometry_px4_msg_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "PX4_ODOMETRY[%.2f]: %.2f s since last message.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_px4_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_px4_msg_->header.stamp)).seconds());
    // Lock the mutex to ensure thread-safe access to the buffer
    std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
    // Insert the new message into the buffer, using its timestamp as the key
    px4_odom_data_.buffer[msg->header.stamp] = msg;
    // Store the pointer to the last message
    last_odometry_px4_msg_ = msg;
  }

  /**
   * @brief Callback for incoming odometry messages from OpenVINS.
   */
  void StateEstimator::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (last_odometry_openvins_msg_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "OPENVINS_ODOMETRY[%.2f]: %.2f s since last message.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_openvins_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_openvins_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
    openvins_odom_data_.buffer[msg->header.stamp] = msg;
    last_odometry_openvins_msg_ = msg;
  }

  /**
   * @brief Callback for incoming odometry messages from Fast-LIO.
   */
  void StateEstimator::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (last_odometry_fast_lio_msg_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "FAST_LIO_ODOMETRY[%.2f]: %.2f s since last message.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_fast_lio_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_fast_lio_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(fast_lio_odom_data_.mtx);
    fast_lio_odom_data_.buffer[msg->header.stamp] = msg;
    last_odometry_fast_lio_msg_ = msg;
  }

  /**
   * @brief Callback for incoming IMU messages.
   */
  void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (last_imu_msg_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "IMU[%.2f]: %.2f s since last message.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(imu_data_.mtx);
    imu_data_.buffer[msg->header.stamp] = msg;
    last_imu_msg_ = msg;
  }

  /**
   * @brief Callback for incoming control diagnostics messages.
   */
  void StateEstimator::controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg)
  {
    if (last_control_input_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "CONTROL[%.2f]: %.2f s since last message.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_control_input_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_control_input_->header.stamp)).seconds());

    // Validate if the control message has the expected size (4 values)
    if (msg->last_control_input.data.size() != 4)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(control_data_.mtx);
    control_data_.buffer[msg->header.stamp] = msg;
    last_control_input_ = msg;
  }

  /**
   * @brief Service callback to set the active odometry source.
   */
  void StateEstimator::setOdometryCallback(
      const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
      std::shared_ptr<laser_msgs::srv::SetString::Response> response)
  {
    RCLCPP_INFO(get_logger(), "SetOdometry service called with request: %s", request->data.c_str());

    // Lock the main mutex to protect shared state (e.g., current_active_odometry_name_)
    std::lock_guard<std::mutex> lock(mtx_);

    const std::string new_source = request->data;

    // 1. Validate if the requested source name is in the list of known sources
    if (std::find(odometry_source_names_.begin(), odometry_source_names_.end(), new_source) == odometry_source_names_.end())
    {
      response->success = false;
      response->message = "Invalid odometry source: '" + new_source + "'. Valid sources are: ";
      for (const auto &name : odometry_source_names_)
      {
        response->message += "'" + name + "' ";
      }
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 2. Check if the requested source is already active
    if (new_source == current_active_odometry_name_)
    {
      response->success = true;
      response->message = "Odometry source '" + new_source + "' is already active.";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 3. Select the data buffer corresponding to the requested source
    SensorDataBuffer<nav_msgs::msg::Odometry> *selected_odom_data = nullptr;
    if (new_source == "openvins_odom")
      selected_odom_data = &openvins_odom_data_;
    else if (new_source == "fast_lio_odom")
      selected_odom_data = &fast_lio_odom_data_;
    else if (new_source == "px4_api_odom")
      selected_odom_data = &px4_odom_data_;

    // 4. Check if the selected source is receiving data
    std::lock_guard<std::mutex> odom_lock(selected_odom_data->mtx);
    if (selected_odom_data->buffer.empty())
    {
      response->success = false;
      response->message = "Switch failed. Odometry buffer for '" + new_source + "' is empty.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 5. Check if the last message from the source has exceeded the timeout
    auto newest_msg_it = selected_odom_data->buffer.rbegin();
    rclcpp::Duration time_since_last_msg = this->get_clock()->now() - newest_msg_it->first;
    if (time_since_last_msg > selected_odom_data->timeout)
    {
      response->success = false;
      response->message = "Switch failed. Timeout on odometry '" + new_source + "'. Last message received " + std::to_string(time_since_last_msg.seconds()) + "s ago.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 6. Check if the new odometry state is close enough to the current EKF state
    const auto &current_state = ekf_->get_state();
    const auto &new_odom_pose = newest_msg_it->second->pose.pose;

    // Calculate position difference
    Eigen::Vector3d current_position(current_state(laser_uav_lib::State::PX), current_state(laser_uav_lib::State::PY), current_state(laser_uav_lib::State::PZ));
    Eigen::Vector3d new_odom_position(new_odom_pose.position.x, new_odom_pose.position.y, new_odom_pose.position.z);
    double distance = (current_position - new_odom_position).norm();

    // Calculate orientation difference
    Eigen::Quaterniond new_orientation(new_odom_pose.orientation.w, new_odom_pose.orientation.x, new_odom_pose.orientation.y, new_odom_pose.orientation.z);
    Eigen::Quaterniond current_orientation(current_state(laser_uav_lib::State::QW), current_state(laser_uav_lib::State::QX), current_state(laser_uav_lib::State::QY), current_state(laser_uav_lib::State::QZ));
    double angle_diff = ekf_->calculate_custom_attitude_error(current_orientation, new_orientation).norm();

    // Calculate linear velocity difference
    Eigen::Vector3d new_odom_linear_velocity(newest_msg_it->second->twist.twist.linear.x, newest_msg_it->second->twist.twist.linear.y, newest_msg_it->second->twist.twist.linear.z);
    Eigen::Vector3d current_linear_velocity(current_state(laser_uav_lib::State::VX), current_state(laser_uav_lib::State::VY), current_state(laser_uav_lib::State::VZ));
    double velocity_diff = (current_linear_velocity - new_odom_linear_velocity).norm();

    // Calculate angular velocity difference
    Eigen::Vector3d new_odom_angular_velocity(newest_msg_it->second->twist.twist.angular.x, newest_msg_it->second->twist.twist.angular.y, newest_msg_it->second->twist.twist.angular.z);
    Eigen::Vector3d current_angular_velocity(current_state(laser_uav_lib::State::WX), current_state(laser_uav_lib::State::WY), current_state(laser_uav_lib::State::WZ));
    double angular_velocity_diff = (current_angular_velocity - new_odom_angular_velocity).norm();

    // Compare the differences with the defined thresholds
    if ((distance > odometry_switch_distance_threshold_) ||
        (angle_diff > odometry_switch_angle_threshold_) ||
        (velocity_diff > odometry_switch_velocity_linear_threshold_) ||
        (angular_velocity_diff > odometry_switch_velocity_angular_threshold_))
    {
      response->success = false;
      response->message = "Switch failed. Odometry '" + new_source + "' is too far from the current estimate. Diffs - Pos: " + std::to_string(distance) + " m, Angle: " + std::to_string(angle_diff) + " rad, LinVel: " + std::to_string(velocity_diff) + " m/s, AngVel: " + std::to_string(angular_velocity_diff) + " rad/s.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 7. Ensure the sensor has been marked as active (i.e., successfully synchronized recently)
    if (!selected_odom_data->is_active)
    {
      response->success = false;
      response->message = "Switch failed. Odometry '" + new_source + "' is not active.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    // If all checks pass, perform the switch
    enable_px4_odom_ = (new_source == "px4_api_odom");
    enable_openvins_odom_ = (new_source == "openvins_odom");
    enable_fast_lio_odom_ = (new_source == "fast_lio_odom");
    current_active_odometry_name_ = new_source;

    response->success = true;
    response->message = "Odometry source switched to: " + current_active_odometry_name_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  /**
   * @brief Retrieves a time-synchronized message from a sensor buffer.
   */
  template <typename MsgT>
  std::optional<MsgT> StateEstimator::getSynchronizedMessage(
      const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name)
  {
    std::lock_guard<std::mutex> lock(sensor_data.mtx);

    // If the buffer is empty, there's nothing to do
    if (sensor_data.buffer.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Message buffer empty.", sensor_name.c_str());
      return std::nullopt;
    }

    // Check if the newest message in the buffer is within the timeout
    auto newest_msg_it = sensor_data.buffer.rbegin();
    if ((ref_time - newest_msg_it->first) > sensor_data.timeout)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Timeout detected. Last msg is %.2f s old. Timeout is %.2f s.", sensor_name.c_str(), (ref_time - newest_msg_it->first).seconds(), sensor_data.timeout.seconds());
      return std::nullopt;
    }

    // Search for the message in the buffer with the smallest time difference from the reference time
    typename std::map<rclcpp::Time, typename MsgT::SharedPtr>::iterator best_match_it = sensor_data.buffer.end();
    rclcpp::Duration min_diff = rclcpp::Duration::max();

    for (auto it = sensor_data.buffer.begin(); it != sensor_data.buffer.end(); ++it)
    {
      rclcpp::Duration diff = ref_time - it->first;
      if (std::abs(diff.seconds()) < min_diff.seconds())
      {
        min_diff = rclcpp::Duration::from_seconds(std::abs(diff.seconds()));
        best_match_it = it;
      }
    }

    if (best_match_it == sensor_data.buffer.end())
      return std::nullopt;

    // If the best match found is within the allowed tolerance, return the message
    if (min_diff <= sensor_data.tolerance)
    {
      // Return a copy of the message for thread safety
      std::optional<MsgT> msg_copy = *best_match_it->second;
      // Mark the sensor as active since it was successfully synchronized
      sensor_data.is_active = true;
      return msg_copy;
    }

    // If the best match is outside the tolerance, reject it and warn the user
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "[%s]: REJECTED: Best match (%.2f ms) is outside tolerance (%.2f ms).",
                         sensor_name.c_str(),
                         min_diff.seconds() * 1000.0,
                         sensor_data.tolerance.seconds() * 1000.0);

    return std::nullopt;
  }

  /**
   * @brief Removes old messages from a sensor buffer.
   */
  template <typename MsgT>
  void StateEstimator::pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data)
  {
    std::lock_guard<std::mutex> lock(sensor_data.mtx);
    if (sensor_data.buffer.empty())
      return;

    // Define a cutoff time. Messages older than this will be removed.
    // We use twice the timeout to keep a safe history.
    const rclcpp::Time cutoff_time = now - (sensor_data.timeout * 2.0);

    // Find the first element in the buffer that is NEWER than the cutoff time.
    auto first_to_keep_it = sensor_data.buffer.upper_bound(cutoff_time);

    // Erase all elements from the beginning of the buffer up to the first one we want to keep.
    sensor_data.buffer.erase(sensor_data.buffer.begin(), first_to_keep_it);
  }

  /**
   * @brief Main timer callback for the EKF update loop.
   */
  void StateEstimator::timerCallback()
  {
    try
    {
      // The loop only runs if the node is in the 'active' state
      if (!is_active_)
        return;
      // The first call just initializes the time
      if (!is_initialized_)
      {
        RCLCPP_INFO(get_logger(), "Initializing EKF...");
        last_update_time_ = this->get_clock()->now();
        is_initialized_ = true;
        return;
      }

      // Set the reference time for this iteration
      rclcpp::Time reference_time = this->get_clock()->now();

      // --- Data Synchronization ---
      // Attempt to get synchronized messages from all sources
      auto px4_odom_msg = getSynchronizedMessage(reference_time, px4_odom_data_, "PX4_ODOMETRY");
      auto openvins_odom_msg = getSynchronizedMessage(reference_time, openvins_odom_data_, "OPENVINS_ODOMETRY");
      auto fast_lio_odom_msg = getSynchronizedMessage(reference_time, fast_lio_odom_data_, "FAST_LIO_ODOMETRY");
      auto imu_msg = getSynchronizedMessage(reference_time, imu_data_, "IMU");
      auto control_msg = getSynchronizedMessage(reference_time, control_data_, "CONTROL");

      // Clean up old messages from the buffers to prevent them from growing indefinitely
      pruneSensorBuffer(reference_time, px4_odom_data_);
      pruneSensorBuffer(reference_time, openvins_odom_data_);
      pruneSensorBuffer(reference_time, fast_lio_odom_data_);
      pruneSensorBuffer(reference_time, imu_data_);
      pruneSensorBuffer(reference_time, control_data_);

      // --- EKF Prediction Step ---
      bool has_prediction{false};
      if (control_msg)
      {
        // For the first control message, just store the time
        if (!is_first_control_msg)
        {
          last_control_input_time_ = rclcpp::Time(control_msg->header.stamp);
          is_first_control_msg = true;
        }
        else
        {
          // Calculate the time interval (dt) since the last prediction
          double dt_sec = (rclcpp::Time(control_msg->header.stamp) - last_control_input_time_).seconds();
          last_update_time_ = rclcpp::Time(control_msg->header.stamp);

          // Ensure time has moved forward and the message is valid
          if (dt_sec >= 0 && control_msg->last_control_input.data.size() == 4)
          {
            // Execute the EKF prediction step with the control input
            Eigen::Vector4d control_input = Eigen::Map<const Eigen::Vector4d>(control_msg->last_control_input.data.data());
            ekf_->predict(control_input, dt_sec);
            // Publish the prediction result (for debugging)
            publishOdometry(predict_pub_);
            last_control_input_time_ = control_msg->header.stamp;
            has_prediction = true;
          }
        }
      }

      // --- EKF Correction Step ---
      laser_uav_lib::MeasurementPackage pkg;
      bool has_measurement{false};
      // Use the measurement from the currently active odometry source
      if (px4_odom_msg && enable_px4_odom_)
      {
        pkg.px4_odometry = *px4_odom_msg;
        has_measurement = true;
      }
      else if (openvins_odom_msg && enable_openvins_odom_)
      {
        pkg.openvins = *openvins_odom_msg;
        has_measurement = true;
      }
      else if (fast_lio_odom_msg && enable_fast_lio_odom_)
      {
        pkg.fast_lio = *fast_lio_odom_msg;
        has_measurement = true;
      }

      // Add the IMU measurement if available
      bool has_measurement_imu{false};
      if (imu_msg)
      {
        pkg.imu = *imu_msg;
        has_measurement_imu = true;
      }

      // If there is any measurement or IMU data, execute the EKF correction step
      if (has_measurement || has_measurement_imu)
        ekf_->correct(pkg);

      // Publish the final state if a prediction or correction occurred
      if ((has_prediction && has_measurement_imu) || has_measurement)
        publishOdometry(odom_pub_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Error in timerCallback: %s", e.what());
    }
  }

  /**
   * @brief Timer callback for publishing diagnostic information.
   */
  void StateEstimator::diagnosticsTimerCallback()
  {
    if (!is_active_)
      return;
    try
    {
      // Create the diagnostics message
      auto diag_msg = std::make_unique<laser_msgs::msg::StateEstimatorDiagnostics>();
      diag_msg->header.stamp = this->get_clock()->now();

      // Set the message's frame_id based on the active odometry's frame
      if (current_active_odometry_name_ == "px4_api_odom" && px4_odom_data_.is_active && !px4_odom_data_.buffer.empty())
        diag_msg->header.frame_id = px4_odom_data_.buffer.begin()->second->header.frame_id;
      else if (current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active && !openvins_odom_data_.buffer.empty())
        diag_msg->header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
      else if (current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active && !fast_lio_odom_data_.buffer.empty())
        diag_msg->header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

      std::lock_guard<std::mutex> lock(mtx_);

      // Fill the general status fields
      diag_msg->active_odometry_source = current_active_odometry_name_;
      diag_msg->is_initialized = is_initialized_;

      // Helper function (lambda) to populate a SensorStatus message
      auto fill_sensor_status = [&](laser_msgs::msg::SensorStatus &status, auto &sensor_data, const std::string &name)
      {
        std::lock_guard<std::mutex> lock(sensor_data.mtx);
        status.name = name;
        status.is_active = sensor_data.is_active;
        status.buffer_size = sensor_data.buffer.size();

        if (!sensor_data.buffer.empty())
        {
          auto last_msg_it = sensor_data.buffer.rbegin();
          status.last_message_stamp = last_msg_it->first;
          status.time_since_last_message = (this->get_clock()->now() - last_msg_it->first).seconds();
          status.has_timeout = (this->get_clock()->now() - last_msg_it->first) > sensor_data.timeout;
        }
        else
        {
          status.has_timeout = true;
          status.time_since_last_message = -1.0;
        }
      };

      // Call the helper function for each data source
      fill_sensor_status(diag_msg->odometry_sources.emplace_back(), px4_odom_data_, "px4_api_odom");
      fill_sensor_status(diag_msg->odometry_sources.emplace_back(), openvins_odom_data_, "openvins_odom");
      fill_sensor_status(diag_msg->odometry_sources.emplace_back(), fast_lio_odom_data_, "fast_lio_odom");
      fill_sensor_status(diag_msg->imu_status, imu_data_, "imu");

      // Publish the diagnostics message
      diagnostics_pub_->publish(std::move(diag_msg));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Error publishing diagnostics: %s", e.what());
    }
  }

  /**
   * @brief Publishes the current EKF state as an Odometry message.
   */
  void StateEstimator::publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub)
  {
    // Get the current state and covariance from the EKF
    const auto &state = ekf_->get_state();
    const auto &cov = ekf_->get_covariance();

    // Safety checks to avoid publishing invalid data
    if (state.size() < 13 || cov.rows() < 13 || cov.cols() < 13)
    {
      RCLCPP_ERROR(get_logger(), "EKF state or covariance has incorrect size.");
      return;
    }
    if (state.hasNaN() || cov.hasNaN())
    {
      RCLCPP_ERROR(get_logger(), "State or covariance contains NaN.");
      return;
    }

    // Create the odometry message
    nav_msgs::msg::Odometry odom_out_msg;
    odom_out_msg.header.stamp = last_update_time_;
    // Set the frame_id based on the active odometry source
    if (current_active_odometry_name_ == "px4_api_odom" && px4_odom_data_.is_active && !px4_odom_data_.buffer.empty())
      odom_out_msg.header.frame_id = px4_odom_data_.buffer.begin()->second->header.frame_id;
    else if (current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active && !openvins_odom_data_.buffer.empty())
      odom_out_msg.header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
    else if (current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active && !fast_lio_odom_data_.buffer.empty())
      odom_out_msg.header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

    // Fill the pose fields (position and orientation)
    odom_out_msg.pose.pose.position.x = state(laser_uav_lib::State::PX);
    odom_out_msg.pose.pose.position.y = state(laser_uav_lib::State::PY);
    odom_out_msg.pose.pose.position.z = state(laser_uav_lib::State::PZ);
    odom_out_msg.pose.pose.orientation.w = state(laser_uav_lib::State::QW);
    odom_out_msg.pose.pose.orientation.x = state(laser_uav_lib::State::QX);
    odom_out_msg.pose.pose.orientation.y = state(laser_uav_lib::State::QY);
    odom_out_msg.pose.pose.orientation.z = state(laser_uav_lib::State::QZ);

    // Fill the twist fields (linear and angular velocities)
    odom_out_msg.twist.twist.linear.x = state(laser_uav_lib::State::VX);
    odom_out_msg.twist.twist.linear.y = state(laser_uav_lib::State::VY);
    odom_out_msg.twist.twist.linear.z = state(laser_uav_lib::State::VZ);
    odom_out_msg.twist.twist.angular.x = state(laser_uav_lib::State::WX);
    odom_out_msg.twist.twist.angular.y = state(laser_uav_lib::State::WY);
    odom_out_msg.twist.twist.angular.z = state(laser_uav_lib::State::WZ);

    // Copy the relevant covariance submatrices from the EKF to the 6x6 ROS message arrays
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        // Pose covariance
        odom_out_msg.pose.covariance[i * 6 + j] = cov(i, j);
        // Twist (velocity) covariance
        odom_out_msg.twist.covariance[i * 6 + j] = cov(i + 7, j + 7);
      }
    }

    // Publish the final message
    pub->publish(odom_out_msg);
  }

} // namespace laser_state_estimator

// Register the node as a component to be loaded dynamically
RCLCPP_COMPONENTS_REGISTER_NODE(laser_state_estimator::StateEstimator)