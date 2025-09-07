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

    double tolerance, timeout;

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
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria PX4 recebida.");
    std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
    px4_odom_data_.buffer[msg->header.stamp] = msg;
  }

  /*//}*/

  /* odometryOpenVinsCallback() //{ */
  void StateEstimator::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria OpenVINS recebida.");
    std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
    openvins_odom_data_.buffer[msg->header.stamp] = msg;
  }
  /*//}*/

  /* odometryFastLioCallback() //{ */
  void StateEstimator::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria FastLIO recebida.");
    std::lock_guard<std::mutex> lock(fast_lio_odom_data_.mtx);
    fast_lio_odom_data_.buffer[msg->header.stamp] = msg;
  }
  /*//}*/

  /* imuCallback() //{ */
  void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de IMU recebida.");
    std::lock_guard<std::mutex> lock(imu_data_.mtx);
    imu_data_.buffer[msg->header.stamp] = msg;
  }
  /*//}*/

  /* controlCallback() //{ */
  void StateEstimator::controlCallback(const laser_msgs::msg::ThrustsMotors::SharedPtr msg)
  {
    if (msg->thrusts.size() != 4)
    {
      RCLCPP_WARN(get_logger(), "Input de controle recebido com tamanho != 4. Ignorando.");
      return;
    }
    std::lock_guard<std::mutex> lock(control_data_.mtx);
    control_data_.buffer[msg->header.stamp] = msg;
    is_control_input_ = true;
  }

  /*//}*/

  template <typename MsgT>
  typename MsgT::SharedPtr StateEstimator::getSynchronizedMessage(
      const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data)
  {
    std::lock_guard<std::mutex> lock(sensor_data.mtx);

    if (sensor_data.buffer.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Buffer de sensor está vazio.");
      return nullptr;
    }

    // Verifica o timeout: a mensagem mais recente no buffer está muito velha?
    auto newest_msg_it = sensor_data.buffer.rbegin();
    if ((ref_time - newest_msg_it->first) > sensor_data.timeout)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Timeout detectado para um sensor. Última msg de %.2f s atrás.", (ref_time - newest_msg_it->first).seconds());
      return nullptr;
    }

    // Encontra a mensagem mais próxima do tempo de referência
    typename MsgT::SharedPtr best_match = nullptr;
    rclcpp::Duration min_diff = rclcpp::Duration::max();

    for (const auto &pair : sensor_data.buffer)
    {
      rclcpp::Duration diff = ref_time - pair.first;
      if (abs(diff.seconds()) < min_diff.seconds())
      {
        min_diff = rclcpp::Duration::from_seconds(abs(diff.seconds()));
        best_match = pair.second;
      }
    }

    // Verifica se a mensagem encontrada está dentro da tolerância
    if (best_match && min_diff <= sensor_data.tolerance)
    {
      return best_match;
    }

    return nullptr; // Nenhuma mensagem encontrada dentro da tolerância
  }

  // Função template para evitar repetição de código
  template <typename MsgT>
  void StateEstimator::pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data)
  {
    std::lock_guard<std::mutex> lock(sensor_data.mtx);
    if (sensor_data.buffer.empty())
    {
      return;
    }

    // 1. Calcula o timestamp mais antigo que queremos manter
    // Usamos o timeout como referência. Qualquer coisa mais antiga que o dobro do timeout será removida.
    const rclcpp::Time cutoff_time = now - (sensor_data.timeout * 2.0);

    // 2. Encontra o primeiro elemento que é MAIS NOVO ou igual ao tempo de corte.
    // Todos os elementos ANTES dele são, portanto, muito antigos.
    auto first_to_erase_it = sensor_data.buffer.upper_bound(cutoff_time);

    // 3. Apaga todos os elementos desde o início do buffer até o primeiro elemento que encontramos.
    // Se todos forem novos, first_to_erase_it será igual a begin() e nada será apagado.
    sensor_data.buffer.erase(sensor_data.buffer.begin(), first_to_erase_it);
  }

  /* timerCallback() //{ */
  void StateEstimator::timerCallback()
  {
    if (!is_active_)
      return;

    // if (!is_control_input_)
    //   return;

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
    last_update_time_ = now;

    // 1. Definir o tempo de referência para este ciclo de fusão
    rclcpp::Time reference_time = this->get_clock()->now();

    // 2. Coletar as mensagens sincronizadas de cada sensor
    auto px4_odom_msg = getSynchronizedMessage(reference_time, px4_odom_data_);
    if (px4_odom_msg)
      pruneSensorBuffer(reference_time, px4_odom_data_);

    auto openvins_odom_msg = getSynchronizedMessage(reference_time, openvins_odom_data_);
    if (openvins_odom_msg)
      pruneSensorBuffer(reference_time, openvins_odom_data_);

    auto fast_lio_odom_msg = getSynchronizedMessage(reference_time, fast_lio_odom_data_);
    if (fast_lio_odom_msg)
      pruneSensorBuffer(reference_time, fast_lio_odom_data_);

    auto imu_msg = getSynchronizedMessage(reference_time, imu_data_);
    if (imu_msg)
      pruneSensorBuffer(reference_time, imu_data_);

    auto control_msg = getSynchronizedMessage(reference_time, control_data_);

    if (control_msg)
    {

      RCLCPP_DEBUG_STREAM(get_logger(), "Mensagem de controle recebida: " << (reference_time - control_msg->header.stamp).seconds() * 1000 << "ms.");
      pruneSensorBuffer(reference_time, control_data_);
      if (control_msg->thrusts.size() == 4)
      {
        ekf_->predict(Eigen::Map<const Eigen::Matrix<double, 4, 1>>(control_msg->thrusts.data()), dt);
        publishOdometry(predict_pub_);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Mensagem de controle recebida com tamanho incorreto: %zu", control_msg->thrusts.size());
      }
    }

    laser_uav_lib::MeasurementPackage pkg;
    if (px4_odom_msg)
    {
      pkg.px4_odometry = *px4_odom_msg;
    }
    if (openvins_odom_msg)
    {
      pkg.fast_lio = *openvins_odom_msg;
    }
    if (fast_lio_odom_msg)
    {
      pkg.openvins = *fast_lio_odom_msg;
    }
    if (imu_msg)
    {
      pkg.imu = *imu_msg;
    }

    if (pkg.px4_odometry || pkg.fast_lio || pkg.openvins || pkg.imu)
      ekf_->correct(pkg);

    if (is_control_input_ || pkg.px4_odometry || pkg.fast_lio || pkg.openvins || pkg.imu)
      publishOdometry(odom_pub_);

    last_odometry_px4_msg_.reset();
    last_imu_msg_.reset();
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