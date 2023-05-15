#include <registration_service/registration_client.hpp>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  std::shared_ptr<registration_client> client = std::make_shared<registration_client>("registration_client_node");
  client->send_test_request();
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}
