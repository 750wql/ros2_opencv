//
// Created by wuqilin on 25-3-9.
//


#include "rclcpp/rclcpp.hpp"
#include "phigros_click.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhigrosClicker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}