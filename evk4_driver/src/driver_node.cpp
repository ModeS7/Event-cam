// Standalone executable wrapper so the component can run via `ros2 run`.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "evk4_driver/evk4_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<evk4_driver::EVK4Driver>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
