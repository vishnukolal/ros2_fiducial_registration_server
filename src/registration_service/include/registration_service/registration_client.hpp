#include <rclcpp/rclcpp.hpp>
#include <registration_msgs/srv/registration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

class registration_client : public rclcpp::Node
{
private:
    rclcpp::Client<registration_msgs::srv::Registration>::SharedPtr reg_client;
    geometry_msgs::msg::Pose ret_pose;
    double ret_error;
    std::string ret_status;

public:
    registration_client(const std::string &name);
    void get_registration(geometry_msgs::msg::Pose& pose, double &error, std::string &status);
    rclcpp::Client<registration_msgs::srv::Registration>::SharedFuture send_request(const std::vector<geometry_msgs::msg::Pose> &fixed_p, const std::vector<geometry_msgs::msg::Pose> &recorded_p, const geometry_msgs::msg::Point &marker_offset);
    void send_test_request();
    void callback(const rclcpp::Client<registration_msgs::srv::Registration>::SharedFuture future);
};
