#pragma once

#include <eigen3/Eigen/Eigen>
#include "../utility/eigen_utility.h"

/** Data structure to store gripper position and orientation.
 */
struct CartesianPose
{
  Eigen::Vector3d position;       ///< the position stored as [x,y,z]
  Eigen::Quaterniond orientation;    ///< the orientation stored as normalized quaternion, [qx,qy,qz,qw], rotation from reference orientation

  static const Eigen::Quaterniond neutralOrientation;   ///< neutral orientation of the gripper, corresponds to [qx,qy,qz,qw] = [1,0,0,0]

  //! constructor, initialize to zero and identity
  CartesianPose();

  //! copy constructor
  CartesianPose(const CartesianPose &rhs);

  //! constructor from affine transformation matrix data, column-major
  CartesianPose(const std::array<double, 16> &pose_TF_as_array);

  //! constructor
  CartesianPose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

  //! constructor
  CartesianPose(const Eigen::Vector3d &translation, const Eigen::AngleAxisd& rotation);

  //! constructor
  CartesianPose(const Eigen::Isometry3d& isometry);

  //! return the values of an affine transformation matrix, column-major
  std::array<double, 16> getHomogenousTransformArray() const;

  //! return a vector 6d with the entries [x,y,z,roll,pitch,yaw]
  Eigen::Vector6d getVector6d() const;

  //! return a vector 7d with the entries [x,y,z,qx,qy,qz,qw]
  Eigen::Vector7d getVector7d() const;

  //! addition operator
  CartesianPose operator+(const CartesianPose &rhs);

  //! addition assignment operator
  CartesianPose &operator+=(const CartesianPose &rhs);

  //! substraction operator
  CartesianPose operator-(const CartesianPose &rhs);

  //! multiplication operator
  CartesianPose operator*(double factor);

  //! compute the difference in translation and rotation to another CartesianPose (rhs-this), result is [dx,dy,dz,dwx,dwy,dwz]
  Eigen::Vector6d getDifferenceTo(const CartesianPose &rhs) const;

  /** \brief Returns the translational distance between this and another `CartesianPose`.
   *
   *  \arg rhs The `CartesianPose` to measure against.
   *  \returns The translational distance.
   * */
  double getTranslationalDistance(const CartesianPose &rhs) const;

  /** \brief Returns the angular distance between this and another `CartesianPose`.
   *
   *  \arg rhs The `CartesianPose` to measure against.
   *  \returns The angular distance.
   * */
  double getRotationalDistance(const CartesianPose &rhs) const;

};

std::ostream &operator<<(std::ostream &stream, const CartesianPose &rhs);
