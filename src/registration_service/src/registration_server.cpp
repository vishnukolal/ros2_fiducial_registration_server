#include <registration_service/registration_server.hpp>
#include <registration_msgs/srv/registration.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

registration_service::registration_service(const std::string &name) : Node(name)
{
  reg_service = create_service<registration_msgs::srv::Registration>("registration_service", std::bind(&registration_service::callback, this, std::placeholders::_1, std::placeholders::_2));
  tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void registration_service::callback(const std::shared_ptr<registration_msgs::srv::Registration::Request> request,
                                    std::shared_ptr<registration_msgs::srv::Registration::Response> response)
{
  if (request->fixed_frames.size() == request->recorded_frames.size())
  {

    positionsCount = request->fixed_frames.size();

    //initialise matrices
    A.resize(positionsCount, 3);
    A.setZero();
    B.resize(positionsCount, 3);
    B.setZero();

    //marker offset is used to change the modify the position of the fixed frames points if recorded points do not actually overlap with the fixed points.
    //for example, if the fixed points are a robot's end effector position, and the recorded points are AR tag positions recorded by a camera, then the marker offset will be the XYZ of the AR tag origin wrt the end effector frame.
    Eigen::Vector3d marker_offset;
    marker_offset.x() = request->marker_offset.x;
    marker_offset.y() = request->marker_offset.y;
    marker_offset.z() = request->marker_offset.z;

    for (int i = 0; i < positionsCount; i++)
    {
      //use marker offset to calculate transformed fixed point wrt origin

      //use orientation of fixed frame in the transformation
      Eigen::Quaterniond q;
      q.x() = request->fixed_frames[i].orientation.x;
      q.y() = request->fixed_frames[i].orientation.y;
      q.z() = request->fixed_frames[i].orientation.z;
      q.w() = request->fixed_frames[i].orientation.w;

      Eigen::Matrix3d offset_rotation = q.normalized().toRotationMatrix();

      //rotate the marker offset with the orientation of each fixed frame
      Eigen::Vector3d marker_offset_rotated = offset_rotation * marker_offset;

      //translate the marker offset with the position of the fixed frame
      A(i, 0) = request->fixed_frames[i].position.x + marker_offset_rotated.x();
      A(i, 1) = request->fixed_frames[i].position.y + marker_offset_rotated.y();
      A(i, 2) = request->fixed_frames[i].position.z + marker_offset_rotated.z();

      // A(i, 0) = request->fixed_frames[i].position.x;
      // A(i, 1) = request->fixed_frames[i].position.y;
      // A(i, 2) = request->fixed_frames[i].position.z;

      //recorded frames points are unchanged
      B(i, 0) = request->recorded_frames[i].position.x;
      B(i, 1) = request->recorded_frames[i].position.y;
      B(i, 2) = request->recorded_frames[i].position.z;
    }

    //use SVD solver to calculate the transformation
    if (SVDSolver(A, B, positionsCount))
    {
      //store transform and registration error in response
      response->transform.position.x = calculatedTranslation.x();
      response->transform.position.y = calculatedTranslation.y();
      response->transform.position.z = calculatedTranslation.z();

      response->transform.orientation.x = calculatedOrientation.x();
      response->transform.orientation.y = calculatedOrientation.y();
      response->transform.orientation.z = calculatedOrientation.z();
      response->transform.orientation.w = calculatedOrientation.w();

      response->info = "success";
      response->error = calculatedError;

      //publish a tf static transform with the calculated transformation
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_link";
      t.child_frame_id = "holo_lens_home";

      t.transform.translation.x = calculatedTranslation.x();
      t.transform.translation.y = calculatedTranslation.y();
      t.transform.translation.z = calculatedTranslation.z();

      t.transform.rotation.x = calculatedOrientation.x();
      t.transform.rotation.y = calculatedOrientation.y();
      t.transform.rotation.z = calculatedOrientation.z();
      t.transform.rotation.w = calculatedOrientation.w();

      tf_static_broadcaster->sendTransform(t);
    }
    else
    {
      response->info = "SVD solver failed";
    }
  }
  else
  {
    response->info = "Size mismatch in fixed and recorded frames";
  }
}

bool registration_service::SVDSolver(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int &positionsCount)
{
  Eigen::MatrixXd A_c; // centered points
  Eigen::MatrixXd B_c;
  Eigen::Vector3d A_mean; // centroid
  Eigen::Vector3d B_mean;

  if (positionsCount <= 0)
  {
    // throw error
    return false;
  }

  // prepare matrices for SVD
  A_c = A.transpose();
  B_c = B.transpose();

  // find centroid of points
  A_mean = A_c.rowwise().sum() / positionsCount;
  B_mean = B_c.rowwise().sum() / positionsCount;

  // demean point set
  for (int i = 0; i < positionsCount; i++)
  {
    A_c.col(i) -= A_mean;
    B_c.col(i) -= B_mean;
  }

  // SVD
  Eigen::MatrixXd S = A_c * B_c.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd VU = svd.matrixV() * svd.matrixU();
  Eigen::Matrix3d D;
  D << 1, 0, 0,
      0, 1, 0,
      0, 0, VU.determinant();

  // calculate rotation and translation from SVD
  calculatedRotationMatrix = svd.matrixV() * D * svd.matrixU().transpose();
  calculatedTranslation = B_mean - calculatedRotationMatrix * A_mean;

  // implicit conversion of rotation matrix to quaternion
  calculatedOrientation = calculatedRotationMatrix;

  // Error calculations
  calculatedError = 0;
  Eigen::MatrixXd transformedPoints;
  Eigen::Vector3d error;
  double FREi = 0;

  // rotate set of points wrt model with computed transformation
  transformedPoints = calculatedRotationMatrix * A.transpose();
  for (int i = 0; i < positionsCount; i++)
  {
    // translate the point with the calculated translation
    transformedPoints.col(i) += calculatedTranslation;

    // find the error between the transformed point and the collected point
    error = B.transpose().col(i) - transformedPoints.col(i);
    FREi = error.norm();
    // square and add errors
    calculatedError += FREi * FREi;
  }
  // find mean of errors
  calculatedError = calculatedError / positionsCount;
  // square root of error is the desired FRE
  calculatedError = sqrt(calculatedError);

  return true;
}