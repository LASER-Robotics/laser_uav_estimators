#include "laser_state_estimator/quadrotor_model_node.hpp"

namespace laser_state_estimator
{
/* QuadrotorModel() //{ */
QuadrotorModel::QuadrotorModel(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("quadrotor_model_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("mass", rclcpp::ParameterValue(2.0));
  declare_parameter("c_tau", rclcpp::ParameterValue(0.59));
  declare_parameter("inertia", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("motors_pos", rclcpp::ParameterValue(std::vector<float_t>(8, 0.0)));

  declare_parameter("rate.pub_state", rclcpp::ParameterValue(100.0));
}
//}

/* ~QuadrotorModel() //{ */
QuadrotorModel::~QuadrotorModel() {
}
//}

/* on_configure() //{ */
CallbackReturn QuadrotorModel::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();

  current_state_ << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  J_.diagonal() << _inertia_x_, _inertia_y_, _inertia_z_;

  for (int i = 0; i < 4; i++) {
    control_input_[i] = 0;
  }

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn QuadrotorModel::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_state_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn QuadrotorModel::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_state_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn QuadrotorModel::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_state_.reset();

  tmr_pub_state_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn QuadrotorModel::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void QuadrotorModel::getParameters() {
  rclcpp::Parameter aux;

  get_parameter("rate.pub_state", _rate_pub_state_);

  get_parameter("mass", _mass_);
  get_parameter("inertia", aux);
  _inertia_x_ = (aux.as_double_array())[0];
  _inertia_y_ = (aux.as_double_array())[1];
  _inertia_z_ = (aux.as_double_array())[2];

  get_parameter("c_tau", _c_tau_);

  get_parameter("motors_pos", aux);
  _motor_position_0_[0] = (aux.as_double_array())[0];
  _motor_position_0_[1] = (aux.as_double_array())[1];
  _motor_position_1_[0] = (aux.as_double_array())[2];
  _motor_position_1_[1] = (aux.as_double_array())[3];
  _motor_position_2_[0] = (aux.as_double_array())[4];
  _motor_position_2_[1] = (aux.as_double_array())[5];
  _motor_position_3_[0] = (aux.as_double_array())[6];
  _motor_position_3_[1] = (aux.as_double_array())[7];
}
//}

/* configPubSub() //{ */
void QuadrotorModel::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");
  sub_control_input_ =
      create_subscription<std_msgs::msg::Float64MultiArray>("uav1/control_input", 1, std::bind(&QuadrotorModel::subControlInput, this, std::placeholders::_1));

  pub_state_ = create_publisher<nav_msgs::msg::Odometry>("predicted_state", 10);
}
//}

/* configTimers() //{ */
void QuadrotorModel::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_pub_state_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_state_), std::bind(&QuadrotorModel::tmrPubState, this), nullptr);
}
//}

/* subControlInput() //{*/
void QuadrotorModel::subControlInput(const std_msgs::msg::Float64MultiArray &msg) {
  if (!is_active_) {
    return;
  }

  double somas = 0;

  for (int i = 0; i < 4; i++) {
    control_input_[i] = msg.data[i];
    somas += msg.data[i];
  }

  Eigen::Quaterniond qm;
  qm.w() = msg.data[4];
  qm.x() = msg.data[5];
  qm.y() = msg.data[6];
  qm.z() = msg.data[7];

  Eigen::Vector3d sim;

  sim << 0, 0, somas;

  Eigen::Vector3d tempo = qm * sim;
  std::cout << tempo << std::endl << std::endl << std::endl;

  _mass_ = tempo(2) / GRAVITY;


  if (control_input_[0] == 0) {
    return;
  }

  deriv_state_     = dynamics(current_state_, control_input_);
  predicted_state_ = predict(current_state_, deriv_state_, 0.001);
  /* predicted_state_ = rk4Integrate(current_state_, control_input_, 0.01); */

  state_.header.stamp            = this->get_clock()->now();
  state_.header.frame_id         = "odom";
  state_.pose.pose.position.x    = predicted_state_(0);
  state_.pose.pose.position.y    = predicted_state_(1);
  state_.pose.pose.position.z    = predicted_state_(2);
  state_.pose.pose.orientation.w = predicted_state_(3);
  state_.pose.pose.orientation.x = predicted_state_(4);
  state_.pose.pose.orientation.y = predicted_state_(5);
  state_.pose.pose.orientation.z = predicted_state_(6);
  state_.twist.twist.linear.x    = predicted_state_(7);
  state_.twist.twist.linear.y    = predicted_state_(8);
  state_.twist.twist.linear.z    = predicted_state_(9);
  state_.twist.twist.angular.x   = predicted_state_(10);
  state_.twist.twist.angular.y   = predicted_state_(11);
  state_.twist.twist.angular.z   = predicted_state_(12);

  pub_state_->publish(state_);

  current_state_ = predicted_state_;
}
//}

/* tmrPubState() //{ */
void QuadrotorModel::tmrPubState() {
  if (!is_active_) {
    return;
  }

  /* if (control_input_[0] == 0) { */
  /*   return; */
  /* } */

  /* deriv_state_ = dynamics(current_state_, control_input_); */
  /* predicted_state_ = predict(current_state_, deriv_state_, 0.01); */

  /* state_.header.stamp            = this->get_clock()->now(); */
  /* state_.header.frame_id         = "odom"; */
  /* state_.pose.pose.position.x    = predicted_state_(0); */
  /* state_.pose.pose.position.y    = predicted_state_(1); */
  /* state_.pose.pose.position.z    = predicted_state_(2); */
  /* state_.pose.pose.orientation.w = predicted_state_(3); */
  /* state_.pose.pose.orientation.x = predicted_state_(4); */
  /* state_.pose.pose.orientation.y = predicted_state_(5); */
  /* state_.pose.pose.orientation.z = predicted_state_(6); */
  /* state_.twist.twist.linear.x    = predicted_state_(7); */
  /* state_.twist.twist.linear.y    = predicted_state_(8); */
  /* state_.twist.twist.linear.z    = predicted_state_(9); */
  /* state_.twist.twist.angular.x   = predicted_state_(10); */
  /* state_.twist.twist.angular.y   = predicted_state_(11); */
  /* state_.twist.twist.angular.z   = predicted_state_(12); */

  /* pub_state_->publish(state_); */

  /* current_state_ = predicted_state_; */
}
//}

/* dynamics() //{ */
QuadrotorModel::Vector13d QuadrotorModel::dynamics(const QuadrotorModel::Vector13d &state, double control_input[]) {
  Eigen::Vector3d    p = state.segment<3>(0);
  Eigen::Quaterniond q(state(3), state(4), state(5), state(6));
  Eigen::Vector3d    vb = state.segment<3>(7);
  Eigen::Vector3d    w  = state.segment<3>(10);

  Eigen::Quaterniond q_norm = q.normalized();

  QuadrotorModel::Vector13d dstate;

  // Position derivative
  dstate.segment<3>(0) = vb;

  Eigen::Quaterniond q_dot;
  // Quaternion derivative
  if (w.norm() < 1e-4) {
    Eigen::Quaterniond bolsonaro(0, 0, 0, 0);
    q_dot = q_norm * bolsonaro;
  } else {
    Eigen::Quaterniond bolsonaro(0, w.x(), w.y(), w.z());
    q_dot = q_norm * bolsonaro;
  }
  q_dot = q_dot.normalized();

  dstate(3) = 0.5 * q_dot.w();
  dstate(4) = 0.5 * q_dot.x();
  dstate(5) = 0.5 * q_dot.y();
  dstate(6) = 0.5 * q_dot.z();

  std::cout << _mass_ << std::endl << std::endl << std::endl;

  // Linear velocity derivative
  double          total_thrust = (control_input[0] + control_input[1] + control_input[2] + control_input[3]) / _mass_;
  Eigen::Vector3d gravity_term = Eigen::Vector3d(0, 0, -GRAVITY);
  Eigen::Vector3d thrust_body(0, 0, total_thrust);

  std::cout << total_thrust << std::endl << std::endl << std::endl;

  /* Eigen::Quaterniond lula(1, 0, 0, 0); */

  dstate.segment<3>(7) = (q_norm.toRotationMatrix() * thrust_body) + gravity_term;
  /* dstate.segment<3>(7) = (lula.toRotationMatrix() * thrust_body) + gravity_term; */

  /* std::cout << q_norm.coeffs() << std::endl << std::endl << std::endl; */

  std::cout << dstate.segment<3>(7) << std::endl << std::endl << std::endl;
  std::cout << _motor_position_0_[0] << "    " << _motor_position_0_[1] << std::endl << std::endl;
  std::cout << _motor_position_1_[0] << "    " << _motor_position_1_[1] << std::endl << std::endl;
  std::cout << _motor_position_2_[0] << "    " << _motor_position_2_[1] << std::endl << std::endl;
  std::cout << _motor_position_3_[0] << "    " << _motor_position_3_[1] << std::endl << std::endl;

  // Total external moments Mb
  Eigen::Vector3d Mb;
  Mb.x() = +control_input[0] * _motor_position_0_[1] + control_input[1] * _motor_position_1_[1] + control_input[2] * _motor_position_2_[1] +
           control_input[3] * _motor_position_3_[1];
  Mb.y() = -control_input[0] * _motor_position_0_[0] - control_input[1] * _motor_position_1_[0] - control_input[2] * _motor_position_2_[0] -
           control_input[3] * _motor_position_3_[0];
  Mb.z() = _c_tau_ * (-control_input[0] + control_input[1] + control_input[2] - control_input[3]);

  /* std::cout << "MB" << Mb << std::endl << std::endl; */

  // Angular velocity derivative
  dstate.segment<3>(10) = J_.inverse() * (Mb - w.cross(J_ * w));

  return dstate;
}
//}

/* predict() //{ */
QuadrotorModel::Vector13d QuadrotorModel::predict(const QuadrotorModel::Vector13d &current_state, const QuadrotorModel::Vector13d &deriv_state, double dt) {


  QuadrotorModel::Vector13d pstate;

  pstate(0) = current_state(0) + deriv_state(0) * dt;
  pstate(1) = current_state(1) + deriv_state(1) * dt;
  pstate(2) = current_state(2) + deriv_state(2) * dt;

  pstate(3) = current_state(3) + deriv_state(3) * dt;
  pstate(4) = current_state(4) + deriv_state(4) * dt;
  pstate(5) = current_state(5) + deriv_state(5) * dt;
  pstate(6) = current_state(6) + deriv_state(6) * dt;
  Eigen::Quaterniond q(pstate(3), pstate(4), pstate(5), pstate(6));
  q         = q.normalized();
  pstate(3) = q.w();
  pstate(4) = q.x();
  pstate(5) = q.y();
  pstate(6) = q.z();

  pstate(7) = current_state(7) + deriv_state(7) * dt;
  pstate(8) = current_state(8) + deriv_state(8) * dt;
  pstate(9) = current_state(9) + deriv_state(9) * dt;

  pstate(10) = current_state(10) + deriv_state(10) * dt;
  pstate(11) = current_state(11) + deriv_state(11) * dt;
  pstate(12) = current_state(12) + deriv_state(12) * dt;

  return pstate;
}

QuadrotorModel::Vector13d QuadrotorModel::rk4Integrate(const Vector13d &current_state, double control_input[], double dt) {

  // RK4 steps
  Vector13d k1 = dynamics(current_state, control_input);
  Vector13d k2 = dynamics(current_state + 0.5 * dt * k1, control_input);
  Vector13d k3 = dynamics(current_state + 0.5 * dt * k2, control_input);
  Vector13d k4 = dynamics(current_state + dt * k3, control_input);

  Vector13d pstate = current_state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  // Normalize quaternion (indices 3 to 6: w, x, y, z)
  Eigen::Quaterniond q(pstate(3), pstate(4), pstate(5), pstate(6));
  q.normalize();
  pstate(3) = q.w();
  pstate(4) = q.x();
  pstate(5) = q.y();
  pstate(6) = q.z();

  return pstate;
}
/* } */
//}

}  // namespace laser_state_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_state_estimator::QuadrotorModel)
