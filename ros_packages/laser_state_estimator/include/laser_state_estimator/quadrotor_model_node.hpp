#ifndef LASER_STATE_ESTIMATOR__QUADROTOR_MODEL_NODE_HPP
#define LASER_STATE_ESTIMATOR__QUADROTOR_MODEL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_state_estimator

#define GRAVITY 9.80665

{
class QuadrotorModel : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit QuadrotorModel(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~QuadrotorModel() override;

  using Vector13d = Eigen::Matrix<double, 13, 1>;
  using Vector8d  = Eigen::Matrix<double, 8, 1>;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();

  void tmrPubState();

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::ConstSharedPtr sub_control_input_;
  void                                                                   subControlInput(const std_msgs::msg::Float64MultiArray &msg);

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_state_;

  Vector13d dynamics(const Vector13d &state, double control_input[]);
  Vector13d predict(const Vector13d &current_state, const Vector13d &deriv_state, double dt);

Vector13d rk4Integrate(const Vector13d &current_state,double control_input[], double dt); 

  bool   is_active_{false};
  double _rate_pub_state_;

  nav_msgs::msg::Odometry      state_;
  rclcpp::TimerBase::SharedPtr tmr_pub_state_;

  Vector13d       previous_state_;
  Vector13d       predicted_state_;
  Vector13d       deriv_state_;
  Vector13d       current_state_;
  Eigen::Matrix3d J_;

  double control_input_[4];

  double _mass_;
  double _motor_position_0_[2];
  double _motor_position_1_[2];
  double _motor_position_2_[2];
  double _motor_position_3_[2];
  double _inertia_x_;
  double _inertia_y_;
  double _inertia_z_;
  double _c_tau_;
};

}  // namespace laser_state_estimator

#endif
