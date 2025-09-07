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

    declare_parameter("process_noise_gains.position", 0.01);
    declare_parameter("process_noise_gains.orientation", 0.01);
    declare_parameter("process_noise_gains.linear_velocity", 0.1);
    declare_parameter("process_noise_gains.angular_velocity", 0.1);

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
    RCLCPP_INFO_STREAM(get_logger(), "  ├ frequency: " << frequency_);

    get_parameter("sensor_timeout", sensor_timeout_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ sensor_timeout: " << sensor_timeout_);

    get_parameter("ekf_verbosity", ekf_verbosity_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ ekf_verbosity: " << ekf_verbosity_);

    get_parameter("drone_params.mass", mass_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ drone_params.mass: " << mass_);

    get_parameter("drone_params.arm_length", arm_length_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ drone_params.arm_length: " << arm_length_);

    get_parameter("drone_params.thrust_coefficient", thrust_coefficient_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ drone_params.thrust_coefficient: " << thrust_coefficient_);

    get_parameter("drone_params.torque_coefficient", torque_coefficient_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ drone_params.torque_coefficient: " << torque_coefficient_);

    get_parameter("drone_params.inertia", inertia_vec_);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ drone_params.inertia: " << "[" << std::to_string(inertia_vec_[0]) << ", " << std::to_string(inertia_vec_[1]) << ", " << std::to_string(inertia_vec_[2]) << "]");

    get_parameter("process_noise_gains.position", process_noise_gains_.position);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ process_noise_gains.position: " << process_noise_gains_.position);

    get_parameter("process_noise_gains.orientation", process_noise_gains_.orientation);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ process_noise_gains.orientation: " << process_noise_gains_.orientation);

    get_parameter("process_noise_gains.linear_velocity", process_noise_gains_.linear_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ process_noise_gains.linear_velocity: " << process_noise_gains_.linear_velocity);

    get_parameter("process_noise_gains.angular_velocity", process_noise_gains_.angular_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ process_noise_gains.angular_velocity: " << process_noise_gains_.angular_velocity);

    get_parameter("measurement_noise_gains.px4_odometry.position", measurement_noise_gains_.px4_odometry.position);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.px4_odometry.position: " << measurement_noise_gains_.px4_odometry.position);

    get_parameter("measurement_noise_gains.px4_odometry.orientation", measurement_noise_gains_.px4_odometry.orientation);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.px4_odometry.orientation: " << measurement_noise_gains_.px4_odometry.orientation);

    get_parameter("measurement_noise_gains.px4_odometry.linear_velocity", measurement_noise_gains_.px4_odometry.linear_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.px4_odometry.linear_velocity: " << measurement_noise_gains_.px4_odometry.linear_velocity);

    get_parameter("measurement_noise_gains.px4_odometry.angular_velocity", measurement_noise_gains_.px4_odometry.angular_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.px4_odometry.angular_velocity: " << measurement_noise_gains_.px4_odometry.angular_velocity);

    get_parameter("measurement_noise_gains.openvins.position", measurement_noise_gains_.openvins.position);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.openvins.position: " << measurement_noise_gains_.openvins.position);

    get_parameter("measurement_noise_gains.openvins.orientation", measurement_noise_gains_.openvins.orientation);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.openvins.orientation: " << measurement_noise_gains_.openvins.orientation);

    get_parameter("measurement_noise_gains.openvins.linear_velocity", measurement_noise_gains_.openvins.linear_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.openvins.linear_velocity: " << measurement_noise_gains_.openvins.linear_velocity);

    get_parameter("measurement_noise_gains.openvins.angular_velocity", measurement_noise_gains_.openvins.angular_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.openvins.angular_velocity: " << measurement_noise_gains_.openvins.angular_velocity);

    get_parameter("measurement_noise_gains.fast_lio.position", measurement_noise_gains_.fast_lio.position);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.fast_lio.position: " << measurement_noise_gains_.fast_lio.position);

    get_parameter("measurement_noise_gains.fast_lio.orientation", measurement_noise_gains_.fast_lio.orientation);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.fast_lio.orientation: " << measurement_noise_gains_.fast_lio.orientation);

    get_parameter("measurement_noise_gains.fast_lio.linear_velocity", measurement_noise_gains_.fast_lio.linear_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.fast_lio.linear_velocity: " << measurement_noise_gains_.fast_lio.linear_velocity);

    get_parameter("measurement_noise_gains.fast_lio.angular_velocity", measurement_noise_gains_.fast_lio.angular_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.fast_lio.angular_velocity: " << measurement_noise_gains_.fast_lio.angular_velocity);

    get_parameter("measurement_noise_gains.imu.angular_velocity", measurement_noise_gains_.imu.angular_velocity);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.imu.angular_velocity: " << measurement_noise_gains_.imu.angular_velocity);

    get_parameter("measurement_noise_gains.gps.position", measurement_noise_gains_.gps.position);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ measurement_noise_gains.gps.position: " << measurement_noise_gains_.gps.position);

    double tolerance, timeout;

    get_parameter("px4_odom_tolerance", tolerance);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ px4_odom_tolerance: " << tolerance << " s");
    get_parameter("px4_odom_timeout", timeout);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ px4_odom_timeout: " << timeout << " s");
    px4_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    px4_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("openvins_odom_tolerance", tolerance);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ openvins_odom_tolerance: " << tolerance << " s");
    get_parameter("openvins_odom_timeout", timeout);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ openvins_odom_timeout: " << timeout << " s");
    openvins_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    openvins_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("fast_lio_odom_tolerance", tolerance);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ fast_lio_odom_tolerance: " << tolerance << " s");
    get_parameter("fast_lio_odom_timeout", timeout);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ fast_lio_odom_timeout: " << timeout << " s");
    fast_lio_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    fast_lio_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("imu_tolerance", tolerance);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ imu_tolerance: " << tolerance << " s");
    get_parameter("imu_timeout", timeout);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ imu_timeout: " << timeout << " s");
    imu_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    imu_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    get_parameter("control_tolerance", tolerance);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ control_tolerance: " << tolerance << " s");
    get_parameter("control_timeout", timeout);
    RCLCPP_INFO_STREAM(get_logger(), "  ├ control_timeout: " << timeout << " s");
    control_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
    control_data_.timeout = rclcpp::Duration::from_seconds(timeout);

    RCLCPP_INFO(get_logger(), "Parâmetros carregados.");
  }
  /*//}*/

  /* configPubSub() //{ */
  void StateEstimator::configPubSub()
  {
    RCLCPP_INFO(get_logger(), "Configurarclcpp::Timendo publishers e subscribers...");
    // Os nomes dos tópicos agora são padrão, para serem remapeados no launch file
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
    predict_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);

    odometry_px4_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&StateEstimator::odometryPx4Callback, this, std::placeholders::_1));
    odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_fast_lio_in", 10, std::bind(&StateEstimator::odometryFastLioCallback, this, std::placeholders::_1));
    odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_openvins_in", 10, std::bind(&StateEstimator::odometryOpenVinsCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", 10, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
    control_sub_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>("control_in", 10, std::bind(&StateEstimator::controlCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Publishers e subscribers configurados.");
  }
  /*//}*/

  /* configTimers() //{ */
  void StateEstimator::configTimers()
  {
    RCLCPP_INFO(get_logger(), "Configurando timers...");
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&StateEstimator::timerCallback, this));

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

    RCLCPP_INFO(get_logger(), "Aplicando ganhos de ruído ao EKF.");

    ekf_->set_process_noise_gains(process_noise_gains_);
    ekf_->set_measurement_noise_gains(measurement_noise_gains_);

    RCLCPP_INFO(get_logger(), "EKF configurado.");
  }
  /*//}*/

  // odometryPx4Callback, imuCallback, controlCallback, timerCallback, publishOdometry permanecem os mesmos...

  /* odometryPx4Callback() //{ */
  void StateEstimator::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria PX4 recebida.");
    if (last_odometry_px4_msg_)
      RCLCPP_INFO(get_logger(), "PX4_ODOMETRY[%.2f]: %.2f s desde a última mensagem.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_px4_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_px4_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
    px4_odom_data_.buffer[msg->header.stamp] = msg;
    last_odometry_px4_msg_ = msg;
  }

  /*//}*/

  /* odometryOpenVinsCallback() //{ */
  void StateEstimator::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria OpenVINS recebida.");
    if (last_odometry_openvins_msg_)
      RCLCPP_INFO(get_logger(), "OPENVINS_ODOMETRY[%.2f]: %.2f s desde a última mensagem.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_openvins_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_openvins_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
    openvins_odom_data_.buffer[msg->header.stamp] = msg;
    last_odometry_openvins_msg_ = msg;
  }
  /*//}*/

  /* odometryFastLioCallback() //{ */
  void StateEstimator::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de odometria FastLIO recebida.");
    if (last_odometry_fast_lio_msg_)
      RCLCPP_INFO(get_logger(), "FAST_LIO_ODOMETRY[%.2f]: %.2f s desde a última mensagem.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_fast_lio_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odometry_fast_lio_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(fast_lio_odom_data_.mtx);
    fast_lio_odom_data_.buffer[msg->header.stamp] = msg;
    last_odometry_fast_lio_msg_ = msg;
  }
  /*//}*/

  /* imuCallback() //{ */
  void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de IMU recebida.");
    if (last_imu_msg_)
      RCLCPP_INFO(get_logger(), "IMU[%.2f]: %.2f s desde a última mensagem.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp)).seconds());
    std::lock_guard<std::mutex> lock(imu_data_.mtx);
    imu_data_.buffer[msg->header.stamp] = msg;
    last_imu_msg_ = msg;
  }
  /*//}*/

  /* controlCallback() //{ */
  void StateEstimator::controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Mensagem de diagnostico de controle recebida.");
    if (last_control_input_)
      RCLCPP_INFO(get_logger(), "CONTROLE[%.2f]: %.2f s desde a última mensagem.", 1 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_control_input_->header.stamp)).seconds(), (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_control_input_->header.stamp)).seconds());

    if (msg->last_control_input.data.size() != 4)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(control_data_.mtx);
    control_data_.buffer[msg->header.stamp] = msg;
    is_control_input_ = true;
    last_control_input_ = msg;
  }
  /*//}*/

  template <typename MsgT>
  std::optional<MsgT> StateEstimator::getSynchronizedMessage(
      const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name)
  {
    std::lock_guard<std::mutex> lock(sensor_data.mtx);

    if (sensor_data.buffer.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Buffer de mensagens vazio.", sensor_name.c_str());
      return std::nullopt; // Retorna opcional vazio
    }

    // A verificação de timeout permanece igual
    auto newest_msg_it = sensor_data.buffer.rbegin();
    if ((ref_time - newest_msg_it->first) > sensor_data.timeout)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Timeout detectado. Última msg de %.2f s atrás. Timeout é %.2f s.", sensor_name.c_str(), (ref_time - newest_msg_it->first).seconds(), sensor_data.timeout.seconds());
      return std::nullopt;
    }

    // Encontra o iterador para a melhor correspondência, não o ponteiro
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
    {
      // Não deve acontecer se o buffer não estiver vazio
      return std::nullopt;
    }

    // Verifica se a melhor correspondência está dentro da tolerância
    if (min_diff <= sensor_data.tolerance)
    {
      // Cria uma cópia da mensagem
      std::optional<MsgT> msg_copy = *best_match_it->second;
      // Remove a mensagem original do buffer
      sensor_data.buffer.erase(best_match_it);
      // Retorna a cópia
      return msg_copy;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "[%s]: REJEITADO: A melhor correspondência (%.2f ms) está fora da tolerância (%.2f ms).",
                         sensor_name.c_str(),
                         min_diff.seconds() * 1000.0,
                         sensor_data.tolerance.seconds() * 1000.0);

    return std::nullopt; // Retorna opcional vazio se estiver fora da tolerância
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

    if (!is_initialized_)
    {
      RCLCPP_INFO(get_logger(), "Inicializando EKF...");
      last_update_time_ = this->get_clock()->now();
      last_cpp_time_point_ = std::chrono::steady_clock::now();
      is_initialized_ = true;
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time reference_time = this->get_clock()->now();

    auto px4_odom_msg = getSynchronizedMessage(reference_time, px4_odom_data_, "PX4_ODOMETRY");
    auto openvins_odom_msg = getSynchronizedMessage(reference_time, openvins_odom_data_, "OPENVINS_ODOMETRY");
    auto fast_lio_odom_msg = getSynchronizedMessage(reference_time, fast_lio_odom_data_, "FAST_LIO_ODOMETRY");
    auto imu_msg = getSynchronizedMessage(reference_time, imu_data_, "IMU");
    auto control_msg = getSynchronizedMessage(reference_time, control_data_, "CONTROL");

    pruneSensorBuffer(reference_time, px4_odom_data_);
    pruneSensorBuffer(reference_time, openvins_odom_data_);
    pruneSensorBuffer(reference_time, fast_lio_odom_data_);
    pruneSensorBuffer(reference_time, imu_data_);
    pruneSensorBuffer(reference_time, control_data_);

    if (control_msg)
    {
      if (!is_first_control_msg)
      {
        last_control_input_time_ = rclcpp::Time(control_msg->header.stamp);
        is_first_control_msg = true;
      }
      else
      {

        double dt = (rclcpp::Time(control_msg->header.stamp) - last_control_input_time_).seconds();
        last_update_time_ = rclcpp::Time(control_msg->header.stamp);

        if (dt <= 0)
        {
          RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Delta time inválido: " << dt << ". Pulando atualização.");
          return;
        }

        if (control_msg->last_control_input.data.size() == 4)
        {
          Eigen::Vector4d control_input = Eigen::Map<const Eigen::Vector4d>(control_msg->last_control_input.data.data());
          ekf_->predict(control_input, dt);
          publishOdometry(predict_pub_);
          last_control_input_time_ = control_msg->header.stamp;
        }
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Mensagem de controle não recebida");
    }

    laser_uav_lib::MeasurementPackage pkg;
    if (px4_odom_msg)
    {
      pkg.px4_odometry = *px4_odom_msg;
      RCLCPP_INFO(get_logger(), "Recebido PX4 Odometry");
    }
    if (openvins_odom_msg)
    {
      pkg.openvins = *openvins_odom_msg;
      RCLCPP_INFO(get_logger(), "Recebido OpenVINS Odometry");
    }
    if (fast_lio_odom_msg)
    {
      pkg.fast_lio = *fast_lio_odom_msg;
      RCLCPP_INFO(get_logger(), "Recebido Fast LIO Odometry");
    }
    if (imu_msg)
    {
      pkg.imu = *imu_msg;
      RCLCPP_INFO(get_logger(), "Recebido IMU");
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
    const auto &state = ekf_->get_state();
    const auto &cov = ekf_->get_covariance();

    nav_msgs::msg::Odometry odom_out_msg;
    odom_out_msg.header.stamp = last_update_time_;
    odom_out_msg.header.frame_id = "odom";
    odom_out_msg.child_frame_id = "uav1/fcu";

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

} // namespace laser_state_estimator

RCLCPP_COMPONENTS_REGISTER_NODE(laser_state_estimator::StateEstimator)