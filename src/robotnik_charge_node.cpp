#include "robotnik_charge/robotnik_charge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotnik_charge::RobotnikCharge>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  executor.spin();

  rclcpp::shutdown();
  return 0;
}