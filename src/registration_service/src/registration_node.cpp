#include <registration_service/registration_server.hpp>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<registration_service>("registration_node"));
  rclcpp::shutdown();

  return 0;
}
