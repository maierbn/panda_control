#pragma once

#include <memory>
#include <array>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include "trajectory.h"
#include "utility/cartesian_pose.h"

/** \brief implements a cartesian linear trajectory between two points.
 *
 * The motion is determined via a polynomial reference trajectory, which is guaranteed to keep the
 * given dynamics limits.
 *
 * \note The trajectory is not time-optimal w.r.t. the dynamics limits.
 *
 * \warning: jerk limits are not yet implemented.
 */
class LinearTrajectory : public Trajectory
{
public:

  /** \brief create a linear trajectory with smooth motion profile between two points with orientations (end effector in
   * robot base coordinate system).
   *
   * \arg[in] startPose starting pose (could be obtained from the robot directly before creation)
   * \arg[in] endPose goal position
   * \arg[in] v_max the maximum allowed velocity [m/s, rad/s]
   * \arg[in] a_max the maximum allowed acceleration [m/s², rad/s²]
   * \arg[in] dt the discretization step width [s].
   */
  LinearTrajectory(const CartesianPose &startPose, const CartesianPose &endPose, double v_max, double a_max, double dt);

  /** \brief get the pose values column-wise for the whole trajectory (end effector in robot base),
   * sampled with dt. */
  std::vector<CartesianPose> poses() const override;

  /** \brief get the pose velocity values column-wise for the whole trajectory (end effector in
   * robot base), sampled with dt. */
  Eigen::Matrix6dynd poseVelocities() const override;

  /** \brief get sample period dt [s] */
  double dt() const override;

  /** \brief get calculated end time [s] */
  double endTime() const override;

protected:

  CartesianPose startPose_;    ///< the starting point and orientation
  CartesianPose endPose_;      ///< the end point and orientation of the trajectory

  double vMax_;     //< maximum allowed velocity
  double aMax_;    //< maximum allowed acceleration
  double dt_;       //< sampling time step width

  double length_;  //< curve length of the trajectory, this is the distance between startPose_ and endPose_
};
