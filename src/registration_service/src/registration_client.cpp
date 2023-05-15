#include <registration_service/registration_client.hpp>
#include <registration_msgs/srv/registration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

registration_client::registration_client(const std::string &name) : Node(name)
{
    reg_client = create_client<registration_msgs::srv::Registration>("/registration_service");
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for server");
    reg_client->wait_for_service();
    RCLCPP_INFO_STREAM(get_logger(), "Ready");
}

void registration_client::send_test_request()
{
    auto request = std::make_shared<registration_msgs::srv::Registration::Request>();

    //Create a matrix A with some example fiducial locations
    Eigen::MatrixXd A(3, 5);
    A(0, 0) = 0;
    A(1, 0) = 5;
    A(2, 0) = 1;

    A(0, 1) = 3;
    A(1, 1) = -5;
    A(2, 1) = 2;

    A(0, 2) = 0;
    A(1, 2) = 1;
    A(2, 2) = 0;

    A(0, 3) = -1;
    A(1, 3) = 3;
    A(2, 3) = 4;

    A(0, 4) = 0;
    A(1, 4) = -1;
    A(2, 4) = -5;

    //create a transformation T (R,p) with some arbitrary position and orientation
    Eigen::Quaterniond q(0.6408564, 0.2988362, -0.0616284, 0.704416);
    Eigen::Vector3d p(-10, 20, 30);
    Eigen::Matrix3d R = q.toRotationMatrix();

    //transform the fiducial locations using T and get another set of points
    //Rotate
    Eigen::MatrixXd B = R * A;

    for (int i = 0; i < B.cols(); i++)
    {
        //Translate
        B.col(i) += p;

        geometry_msgs::msg::Pose poseA;
        poseA.position.x = A(0, i);
        poseA.position.y = A(1, i);
        poseA.position.z = A(2, i);

        geometry_msgs::msg::Pose poseB;
        poseB.position.x = B(0, i);
        poseB.position.y = B(1, i);
        poseB.position.z = B(2, i);

        //pack pose message with the points. Pose message orientation is unused.
        request->fixed_frames.push_back(poseA);
        request->recorded_frames.push_back(poseB);
    } 

    //send the two sets of points to server and recover the transformation T.
    reg_client->async_send_request(request, std::bind(&registration_client::callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(get_logger(), "Request sent");
}

rclcpp::Client<registration_msgs::srv::Registration>::SharedFuture registration_client::send_request(const std::vector<geometry_msgs::msg::Pose> &fixed_p, const std::vector<geometry_msgs::msg::Pose> &recorded_p, const geometry_msgs::msg::Point &marker_offset)
{
    auto request = std::make_shared<registration_msgs::srv::Registration::Request>();

    request->fixed_frames = fixed_p;
    request->recorded_frames = recorded_p;
    request->marker_offset = marker_offset;

    auto result = reg_client->async_send_request(request, std::bind(&registration_client::callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(get_logger(), "Registration request sent");
    return result;
}

void registration_client::callback(const rclcpp::Client<registration_msgs::srv::Registration>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO_STREAM(get_logger(), "Response received");

    RCLCPP_INFO_STREAM(get_logger(), "Status: " << response->info);

    RCLCPP_INFO_STREAM(get_logger(), "Pos X: " << response->transform.position.x);
    RCLCPP_INFO_STREAM(get_logger(), "Pos Y: " << response->transform.position.y);
    RCLCPP_INFO_STREAM(get_logger(), "Pos Z: " << response->transform.position.z);

    RCLCPP_INFO_STREAM(get_logger(), "Ori X: " << response->transform.orientation.x);
    RCLCPP_INFO_STREAM(get_logger(), "Ori Y: " << response->transform.orientation.y);
    RCLCPP_INFO_STREAM(get_logger(), "Ori Z: " << response->transform.orientation.z);
    RCLCPP_INFO_STREAM(get_logger(), "Ori W: " << response->transform.orientation.w);

    RCLCPP_INFO_STREAM(get_logger(), "Registration error: " << response->error);

    ret_pose = response->transform;
    ret_status = response->info;
    ret_error = response->error;
}

void registration_client::get_registration(geometry_msgs::msg::Pose& pose, double &error, std::string &status)
{
    pose = ret_pose;
    error = ret_error;
    status = ret_status;
}
