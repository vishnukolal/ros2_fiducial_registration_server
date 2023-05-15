#include <rclcpp/rclcpp.hpp>
#include <registration_msgs/srv/registration.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

class registration_service : public rclcpp::Node
{
private:
  rclcpp::Service<registration_msgs::srv::Registration>::SharedPtr reg_service;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  int positionsCount;

  Eigen::Matrix3d calculatedRotationMatrix;
  Eigen::Quaterniond calculatedOrientation;
  Eigen::Vector3d calculatedTranslation;
  double calculatedError;

  bool SVDSolver(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int &positionsCount);

public:
  registration_service(const std::string &name);
  void callback(const std::shared_ptr<registration_msgs::srv::Registration::Request> request,
                std::shared_ptr<registration_msgs::srv::Registration::Response> response);
};
