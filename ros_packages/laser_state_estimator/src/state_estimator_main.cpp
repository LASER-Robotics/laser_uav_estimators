#include <rclcpp/rclcpp.hpp>
#include "laser_state_estimator/state_estimator_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laser_state_estimator::StateEstimator>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}