#include <laser_state_estimator/state_estimator_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_state_estimator
{
  /* StateEstimator() //{ */
  StateEstimator::StateEstimator(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("state_estimator", options)
  {
    RCLCPP_INFO(get_logger(), "Criando StateEstimator...");

    // Declaração de parâmetros (tópicos foram removidos)
    declare_parameter("frequency", 100.0);
    declare_parameter("sensor_timeout", 0.5);
    declare_parameter("ekf_verbosity", "INFO");
    declare_parameter("drone_params.mass", 1.0);
    declare_parameter("drone_params.arm_length", 0.25);
    declare_parameter("drone_params.thrust_coefficient", 1.0);
    declare_parameter("drone_params.torque_coefficient", 0.05);
    declare_parameter("drone_params.inertia", std::vector<double>{0.01, 0.01, 0.01});
    declare_parameter("process_noise", std::vector<double>(13, 0.01));

    last_control_input_.setZero();
    RCLCPP_INFO(get_logger(), "StateEstimator node initialized.");
  }
  /*//}*/

  /* ~StateEstimator() //{ */
  StateEstimator::~StateEstimator() {}
  /*//}*/

  /* on_configure() //{ */
  CallbackReturn StateEstimator::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configurando StateEstimator...");

    getParameters();
    configPubSub();
    configTimers();
    configServices();
    setupEKF(); // EKF setup pode vir depois dos parâmetros

    return CallbackReturn::SUCCESS;
  }
  /*//}*/

  // on_activate, on_deactivate, on_cleanup, on_shutdown permanecem os mesmos...

  /* on_activate() //{ */
  CallbackReturn StateEstimator::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Ativando StateEstimator...");
    odom_pub_->on_activate();
    predict_pub_->on_activate();
    is_active_ = true;
    timer_->reset();
    check_subscribers_timer_->reset();
    ekf_->reset();
    return CallbackReturn::SUCCESS;
  }
  /*//}*/

  /* on_deactivate() //{ */
  CallbackReturn StateEstimator::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Desativando StateEstimator...");
    is_active_ = false;
    timer_->cancel();
    check_subscribers_timer_->cancel();
    odom_pub_->on_deactivate();
    predict_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }
  /*//}*/

  /* on_cleanup() //{ */
  CallbackReturn StateEstimator::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Limpando StateEstimator...");
    ekf_.reset();
    odom_pub_.reset();
    predict_pub_.reset();

    odometry_px4_sub_.reset();
    odometry_fast_lio_sub_.reset();
    odometry_openvins_sub_.reset();
    imu_sub_.reset();
    control_sub_.reset();
    timer_.reset();
    check_subscribers_timer_.reset();
    return CallbackReturn::SUCCESS;
  }
  /*//}*/

  /* on_shutdown() //{ */
  CallbackReturn StateEstimator::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Encerrando StateEstimator...");
    return CallbackReturn::SUCCESS;
  }
  /*//}*/

  /* getParameters() //{ */
  void StateEstimator::getParameters()
  {
    RCLCPP_INFO(get_logger(), "Carregando parâmetros...");
    get_parameter("frequency", frequency_);
    get_parameter("sensor_timeout", sensor_timeout_);
    get_parameter("ekf_verbosity", ekf_verbosity_);
    get_parameter("drone_params.mass", mass_);
    get_parameter("drone_params.arm_length", arm_length_);
    get_parameter("drone_params.thrust_coefficient", thrust_coefficient_);
    get_parameter("drone_params.torque_coefficient", torque_coefficient_);
    get_parameter("drone_params.inertia", inertia_vec_);
    get_parameter("process_noise", process_noise_vec_);
    RCLCPP_INFO(get_logger(), "Parâmetros carregados.");
  }
  /*//}*/

  /* configPubSub() //{ */
  void StateEstimator::configPubSub()
  {
    RCLCPP_INFO(get_logger(), "Configurando publishers e subscribers...");
    // Os nomes dos tópicos agora são padrão, para serem remapeados no launch file
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
    predict_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);

    odometry_px4_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&StateEstimator::odometryPx4Callback, this, std::placeholders::_1));
    odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_fast_lio_in", 10, std::bind(&StateEstimator::odometryFastLioCallback, this, std::placeholders::_1));
    odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_openvins_in", 10, std::bind(&StateEstimator::odometryOpenVinsCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", 10, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
    control_sub_ = create_subscription<laser_msgs::msg::ThrustsMotors>("control_in", 10, std::bind(&StateEstimator::controlCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Publishers e subscribers configurados.");
  }
  /*//}*/

  /* configTimers() //{ */
  void StateEstimator::configTimers()
  {
    RCLCPP_INFO(get_logger(), "Configurando timers...");
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&StateEstimator::timerCallback, this));
    timer_->cancel(); // O timer só começa a correr no on_activate

    check_subscribers_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&StateEstimator::checkSubscribersCallback, this));
    check_subscribers_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Timers configurados.");
  }
  /*//}*/

  /* configServices() //{ */
  void StateEstimator::configServices()
  {
    RCLCPP_INFO(get_logger(), "Configurando serviços... (Nenhum serviço a ser configurado)");
  }
  /*//}*/

  /* setupEKF() //{ */
  void StateEstimator::setupEKF()
  {
    RCLCPP_INFO(get_logger(), "Configurando EKF...");
    Eigen::Matrix3d inertia = Eigen::Vector3d(inertia_vec_[0], inertia_vec_[1], inertia_vec_[2]).asDiagonal();

    ekf_ = std::make_unique<laser_uav_lib::DroneEKF>(mass_, arm_length_, thrust_coefficient_, torque_coefficient_, inertia, ekf_verbosity_);

    Eigen::Matrix<double, 13, 13> Q = Eigen::Matrix<double, 13, 13>::Zero();
    for (int i = 0; i < 13; ++i)
    {
      Q(i, i) = process_noise_vec_[i];
    }
    ekf_->set_process_noise(Q);
    RCLCPP_INFO(get_logger(), "EKF configurado.");
  }
  /*//}*/

  // odometryPx4Callback, imuCallback, controlCallback, timerCallback, publishOdometry permanecem os mesmos...

  /* odometryPx4Callback() //{ */
  void StateEstimator::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Mensagem de odometria PX4 recebida.");
    std::lock_guard<std::mutex> lock(mtx_);
    last_odometry_px4_msg_ = msg;
    last_px4_odom_time_ = this->get_clock()->now();
  }
  /*//}*/

  /* odometryOpenVinsCallback() //{ */
  void StateEstimator::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Mensagem de odometria OpenVINS recebida.");
    std::lock_guard<std::mutex> lock(mtx_);
    last_odometry_openvins_msg_ = msg;
    last_openvins_odom_time_ = this->get_clock()->now();
  }
  /*//}*/

  /* odometryFastLioCallback() //{ */
  void StateEstimator::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Mensagem de odometria FastLIO recebida.");
    std::lock_guard<std::mutex> lock(mtx_);
    last_odometry_fast_lio_msg_ = msg;
    last_fast_lio_odom_time_ = this->get_clock()->now();
  }
  /*//}*/

  /* imuCallback() //{ */
  void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_imu_msg_ = msg;
    last_imu_time_ = this->get_clock()->now();
  }
  /*//}*/

  /* controlCallback() //{ */
  void StateEstimator::controlCallback(const laser_msgs::msg::ThrustsMotors::SharedPtr msg)
  {
    if (msg->thrusts.size() != 4)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Input de controle recebido com tamanho != 4. Ignorando.");
      return;
    }
    std::lock_guard<std::mutex> lock(mtx_);

    for (int i = 0; i < 4; ++i)
      last_control_input_(i) = msg->thrusts[i];

    last_control_input_time_ = this->get_clock()->now();

    is_control_input_ = true;
  }
  /*//}*/

  /* timerCallback() //{ */
  void StateEstimator::timerCallback()
  {
    if (!is_active_)
      return;

    if (!is_initialized_)
    {
      RCLCPP_INFO(get_logger(), "Inicializando EKF...");
      last_update_time_ = this->get_clock()->now();
      is_initialized_ = true;
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_update_time_).seconds();
    if (dt <= 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Delta time zero ou negativo. A saltar um ciclo de predição.");
      return;
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "Cálculo de Tempo:\n"
                           << "  - Now          : " << now.seconds() << " s\n"
                           << "  - Last Update  : " << last_update_time_.seconds() << " s\n"
                           << "  - Delta Time   : " << dt << " s\n"
                           << "  - Freq. Medida : " << 1.0 / dt << " Hz");

    RCLCPP_INFO_STREAM(get_logger(), "Frequency TimerCallback[" << 1.0 / frequency_ << "]: " << 1 / dt << " Hz - " << dt << " s");
    last_update_time_ = now;

    if (!is_control_input_)
      return;

    { // Lock scope
      std::lock_guard<std::mutex> lock(mtx_);
      if (is_control_input_)
      {
        ekf_->predict(last_control_input_, dt);
        publishOdometry(predict_pub_);
      }

      laser_uav_lib::MeasurementPackage pkg;
      if (last_odometry_px4_msg_)
        pkg.px4_odometry = *last_odometry_px4_msg_;
      if (last_odometry_fast_lio_msg_)
        pkg.fast_lio = *last_odometry_fast_lio_msg_;
      if (last_odometry_openvins_msg_)
        pkg.openvins = *last_odometry_openvins_msg_;
      if (last_imu_msg_)
        pkg.imu = *last_imu_msg_;

      if (pkg.px4_odometry || pkg.imu)
        ekf_->correct(pkg);

      if (is_control_input_ || pkg.px4_odometry)
        publishOdometry(odom_pub_);

      last_odometry_px4_msg_.reset();
      last_imu_msg_.reset();
    }
  }
  /*//}*/

  /* publishOdometry() //{ */
  void StateEstimator::publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub)
  {
    RCLCPP_INFO(get_logger(), "Publicando mensagem de odometria.");
    const auto &state = ekf_->get_state();
    const auto &cov = ekf_->get_covariance();

    nav_msgs::msg::Odometry odom_out_msg;
    odom_out_msg.header.stamp = last_update_time_;
    odom_out_msg.header.frame_id = "odom";
    odom_out_msg.child_frame_id = "base_link";

    odom_out_msg.pose.pose.position.x = state(laser_uav_lib::State::PX);
    odom_out_msg.pose.pose.position.y = state(laser_uav_lib::State::PY);
    odom_out_msg.pose.pose.position.z = state(laser_uav_lib::State::PZ);
    odom_out_msg.pose.pose.orientation.w = state(laser_uav_lib::State::QW);
    odom_out_msg.pose.pose.orientation.x = state(laser_uav_lib::State::QX);
    odom_out_msg.pose.pose.orientation.y = state(laser_uav_lib::State::QY);
    odom_out_msg.pose.pose.orientation.z = state(laser_uav_lib::State::QZ);

    odom_out_msg.twist.twist.linear.x = state(laser_uav_lib::State::VX);
    odom_out_msg.twist.twist.linear.y = state(laser_uav_lib::State::VY);
    odom_out_msg.twist.twist.linear.z = state(laser_uav_lib::State::VZ);
    odom_out_msg.twist.twist.angular.x = state(laser_uav_lib::State::WX);
    odom_out_msg.twist.twist.angular.y = state(laser_uav_lib::State::WY);
    odom_out_msg.twist.twist.angular.z = state(laser_uav_lib::State::WZ);

    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        odom_out_msg.pose.covariance[i * 6 + j] = cov(i, j);
        odom_out_msg.twist.covariance[i * 6 + j] = cov(i + 7, j + 7);
      }
    }

    pub->publish(odom_out_msg);
  }
  /*//}*/

  /**
   * @brief Verifica periodicamente a atividade dos subscribers de sensores.
   * Se uma mensagem não for recebida dentro de um tempo limite ('sensor_timeout_sec'),
   * a última mensagem armazenada é invalidada (definida como nullptr) para que
   * o EKF pare de usá-la.
   */
  void StateEstimator::checkSubscribersCallback()
  {
    // Apenas executa se o nó estiver ativo
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      return;
    }

    // Trava o mutex para acessar as variáveis de forma segura
    std::scoped_lock lock(mtx_);

    const auto now = this->get_clock()->now();

    double sensor_timeout_sec_ = 1 / sensor_timeout_;

    // --- Verifica Odometria da PX4 ---
    if (last_odometry_px4_msg_ != nullptr)
    {
      if ((now - last_px4_odom_time_).seconds() > sensor_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Timeout na odometria da PX4. A última mensagem foi recebida há mais de %.1f segundos. Descartando dados.", sensor_timeout_sec_);
        last_odometry_px4_msg_ = nullptr; // Invalida a mensagem
      }
    }

    // --- Verifica Odometria do OpenVINS ---
    if (last_odometry_openvins_msg_ != nullptr)
    {
      if ((now - last_openvins_odom_time_).seconds() > sensor_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Timeout na odometria do OpenVINS. A última mensagem foi recebida há mais de %.1f segundos. Descartando dados.", sensor_timeout_sec_);
        last_odometry_openvins_msg_ = nullptr; // Invalida a mensagem
      }
    }

    // --- Verifica Odometria do FastLIO ---
    if (last_odometry_fast_lio_msg_ != nullptr)
    {
      if ((now - last_fast_lio_odom_time_).seconds() > sensor_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Timeout na odometria do FastLIO. A última mensagem foi recebida há mais de %.1f segundos. Descartando dados.", sensor_timeout_sec_);
        last_odometry_fast_lio_msg_ = nullptr; // Invalida a mensagem
      }
    }

    // --- Verifica IMU ---
    if (last_imu_msg_ != nullptr)
    {
      if ((now - last_imu_time_).seconds() > sensor_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Timeout no IMU. A última mensagem foi recebida há mais de %.1f segundos. Descartando dados.", sensor_timeout_sec_);
        last_imu_msg_ = nullptr; // Invalida a mensagem
      }
    }
  }

} // namespace laser_state_estimator

RCLCPP_COMPONENTS_REGISTER_NODE(laser_state_estimator::StateEstimator)