#include "trajectory_plotter.h"

#include <iostream>
#include <eigen3/Eigen/Eigen>

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth)
{
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  int nEntries = poseVelocities.size();
  std::cout << "nEntries: " << nEntries << std::endl;

  const double dt = 0.1;

  //Eigen::Matrix5dynd 

  CartesianPose currentPose;
  for (int i = 0; i < nEntries; i++)
  {
    Eigen::Vector3d pose = posVelocities.col(i);
    
    double roll = pose[3];
    double pitch = pose[4];
    double yaw = pose[5];

    Quaterniond rotation;
    rotation = AngleAxisd(roll, Vector3f::UnitX())
        * AngleAxisd(pitch, Vector3f::UnitY())
        * AngleAxisd(yaw, Vector3f::UnitZ());
    
    Eigen::Vector3d velocity = pose.head<3>();

    currentPose.position += velocity * dt;
    currentPose.orientation *= rotation * dt;
  }
}

void TrajectoryPlotter::plot()
{
  
}
