#include "utility/cartesian_pose.h"

#include <iostream>

const Eigen::Quaterniond CartesianPose::neutralOrientation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0).normalized();

CartesianPose::CartesianPose()
{
  position << 0,0,0;
  orientation = neutralOrientation;
}

CartesianPose::CartesianPose(const CartesianPose &rhs) : position(rhs.position), orientation(rhs.orientation)
{
}

CartesianPose::CartesianPose(const std::array<double, 16> &poseTransformationMatrix)
{
  // Parameter is e.g. from franka::RobotState::O_T_EE
  // see: https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#a193781d47722b32925e0ea7ac415f442
  // Pose is represented as a 4x4 matrix in column-major format. 

  // store the 4x4 transformation matrix as Eigen data structure
  Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
  transformation.matrix() = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>(poseTransformationMatrix.data());

  orientation = transformation.rotation();
  position = transformation.translation();

  //std::cout << "initialize CartesianPose from transformation matrix: \n\ttranslation: " << position.transpose() << std::endl;
  //std::cout << "\torientation: (qx,qy,qz,qw) = " << orientation.coeffs().transpose() << ", Euler angles: " << orientation.toRotationMatrix().eulerAngles(0,1,2).transpose() << std::endl;
    
}

CartesianPose::CartesianPose(const Eigen::Vector3d &_position, const Eigen::Quaterniond &_orientation) : position(_position), orientation(_orientation)
{
}

CartesianPose::CartesianPose(const Eigen::Vector3d &translation,
                             const Eigen::AngleAxisd& rotation) : CartesianPose(translation, Eigen::Quaterniond(rotation)){}

CartesianPose::CartesianPose(const Eigen::Isometry3d& isometry) : CartesianPose(isometry.translation(), Eigen::AngleAxisd(isometry.rotation())){}



std::array<double, 16> CartesianPose::getHomogenousTransformArray() const
{
  Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
  transformation.translation() = position;
  transformation.rotate(orientation);

  std::array<double, 16> result;
  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Map(result.data()) = transformation.matrix();

  return result;
}

Eigen::Vector6d CartesianPose::getVector6d() const
{
  Eigen::Vector6d result;

  // initialize the vector values x,y,z, roll,pitch,yaw
  result << position, orientation.toRotationMatrix().eulerAngles(0,1,2);
  return result;
}

Eigen::Vector7d CartesianPose::getVector7d() const
{
  Eigen::Vector7d result;

  // initialize the vector values x,y,z, qx,qy,qz,qw
  result << position, orientation.coeffs();
  return result;
}

// addition operator
CartesianPose CartesianPose::operator+(const CartesianPose &rhs)
{
  CartesianPose result(*this);
  result.position += rhs.position;
  result.orientation.normalize();
  result.orientation *= rhs.orientation;

  return result;
}

//! addition assignment operator
CartesianPose &CartesianPose::operator+=(const CartesianPose &rhs)
{
  position += rhs.position;
  orientation.normalize();
  orientation *= rhs.orientation;

  return *this;
}

// substraction operator
CartesianPose CartesianPose::operator-(const CartesianPose &rhs)
{
  CartesianPose result(*this);
  result.position -= rhs.position;
  result.orientation.normalize();
  result.orientation *= rhs.orientation.conjugate();  // reverse orientation

  return result;
}

//! multiplication operator
CartesianPose CartesianPose::operator*(double factor)
{
  CartesianPose result(*this);
  result.position *= factor;
  result.orientation.normalize();
  result.orientation = neutralOrientation.slerp(factor, orientation);  // scale rotation from neutral to current orientation

  return result;
}

Eigen::Vector6d CartesianPose::getDifferenceTo(const CartesianPose &rhs) const
{
  Eigen::Vector6d difference;

  // compute translation
  difference.head<3>() = rhs.position - position;

  // Compute the rotation from orientation1 to orientation2.
  // Determine the Euler-angles for (orientation1 to orientation2) and the negative EUler-angles for (orientation2 to orientation1).
  // Use the set of Euler-angles with the smaller norm.

  // compute orientation
  Eigen::Quaterniond rotation = rhs.orientation * orientation.inverse();

  Eigen::Vector3d eulerAnglesForward = rotation.toRotationMatrix().eulerAngles(0,1,2);
  Eigen::Vector3d eulerAnglesBackward = -rotation.inverse().toRotationMatrix().eulerAngles(0,1,2);

  // select the euler angles with smaller norm
  difference.tail<3>() = eulerAnglesForward;
  if (eulerAnglesBackward.norm() < eulerAnglesForward.norm())
  {
    difference.tail<3>() = eulerAnglesBackward;
  }

  return difference;
}

double CartesianPose::getTranslationalDistance(const CartesianPose &other) const {
  return (position - other.position).norm();
}

double CartesianPose::getRotationalDistance(const CartesianPose &other) const {
  return orientation.angularDistance(other.orientation);
}

std::ostream &operator<<(std::ostream &stream, const CartesianPose &rhs)
{
  stream << "xyz: (" << rhs.position[0] << "," << rhs.position[1] << "," << rhs.position[2] << ")  ypr: ("
    << rhs.orientation.toRotationMatrix().eulerAngles(2,1,0).transpose() / M_PI * 180. << ") q xyzw: (" << rhs.orientation.coeffs().transpose() << ")";
  return stream;
}