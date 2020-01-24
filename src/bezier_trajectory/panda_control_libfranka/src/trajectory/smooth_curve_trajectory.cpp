#include "smooth_curve_trajectory.h"

#include <iomanip>
#include <iostream>

SmoothCurveTrajectory::SmoothCurveTrajectory(const CartesianPose initialPose,
  std::function<CartesianPose (double t)> curve, double endTime, double dt) :
  curveSmoothener_(curve, endTime), curveTrajectory_(initialPose, curveSmoothener_.smoothCurve(), endTime, dt)
{
}

/** \brief get the pose values column-wise for the whole trajectory (end effector in robot base),
  * sampled with dt. */
std::vector<CartesianPose> SmoothCurveTrajectory::poses() const
{
  return curveTrajectory_.poses();
}

/** \brief get the pose velocity values column-wise for the whole trajectory (end effector in
  * robot base), sampled with dt. */
Eigen::Matrix6dynd SmoothCurveTrajectory::poseVelocities()
{
  return curveTrajectory_.poseVelocities();
}

/** \brief get sample period dt [s] */
double SmoothCurveTrajectory::dt() const
{
  return curveTrajectory_.dt();
}

/** \brief get calculated end time [s] */
double SmoothCurveTrajectory::endTime() const
{
  return curveTrajectory_.endTime();
}
