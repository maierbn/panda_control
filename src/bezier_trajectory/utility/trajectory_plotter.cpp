#include "trajectory_plotter.h"

#include <iostream>

#include "utility/eigen_utility.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

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
    /*Eigen::Vector3d pose = poseVelocities.col(i);
    
    double roll = pose[3] * dt;
    double pitch = pose[4] * dt;
    double yaw = pose[5] * dt;

    Eigen::Quaterniond rotation;
    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Vector3d velocity = pose.head<3>();

    currentPose.position += velocity * dt;
    currentPose.orientation *= rotation;
    */
  }
}

void TrajectoryPlotter::plot()
{
  
}
