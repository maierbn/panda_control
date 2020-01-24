#pragma once

#include "utility/cartesian_pose.h"
#include "trajectory/trajectory.h"

class TrajectoryPlotter
{
public:

  //! construct the plotter and plot the trajectory
  TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth = 1.e-3);

  //! plot the trajectory and additionally the given poses
  TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory,
                    const std::vector<CartesianPose> &poses, double samplingTimestepWidth = 1.e-3);

  //! plot the trajectory and additionally the given poses, also bezier interpolation for each component
  TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory,
                    const std::vector<CartesianPose> &poses, const std::vector<double> &knots, double samplingTimestepWidth = 1.e-3);

private:
  //! compute and plot velocity, acceleration and jerk over time
  std::string plotPoseVelocities(const Eigen::Matrix6dynd &poseVelocities, double dt);

  //! plot the trajectory and gripper orientation in a 3D view, if poses is not empty also plot the poses as point
  std::string plotTrajectory(CartesianPose startPose, const Eigen::Matrix6dynd &poseVelocities, double dt,
                             const std::vector<CartesianPose> &poses, const std::vector<double> &knots);
};
