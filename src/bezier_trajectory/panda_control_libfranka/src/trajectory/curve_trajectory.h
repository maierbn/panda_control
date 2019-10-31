#pragma once

#include <memory>
#include <array>
#include <functional>
#include <utility>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include "trajectory.h"
#include "utility/cartesian_pose.h"

/** \brief implements a trajectory following an arbitrary curve ∈ ℝ^3
 */
class CurveTrajectory : public Trajectory
{
public:

  /** \brief create a linear trajectory witgh motion profile between to points (end effector in
   * robot base coordinate system).
   *
   * \arg[in] initialPose starting pose (could be obtained from the robot directly before creation)
   *          [x,y,z, R, P, Y], position in [m], orientation in [rad]
   * \arg[in] curve trajectory curve, offset by initialPose [x,y,z]
   * \arg[in] dt the discretization step width [s]. Must match the execution times in the robot
   * control.
   */
  CurveTrajectory(const CartesianPose initialPose, std::function<CartesianPose (double t)> curve, double endTime, double dt);

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
  double endTime_;  // duration of trajectory
  double dt_;  // timstep width or sampling width of the trajectory
  CartesianPose initialPose_;  // initial pose from where to start trajectory
  CartesianPose curveStartPose_;   // initial point of the curve

  std::function<CartesianPose (double t)> curve_;   // curve that describes the trajectory
};
