#include "trajectory_plotter.h"

#include <iostream>

#include "utility/eigen_utility.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <fstream>

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth)
{
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  int nEntries = poseVelocities.size();
  std::cout << "nEntries: " << nEntries << std::endl;

  const double dt = samplingTimestepWidth;

  std::stringstream pythonScript;

  pythonScript << "#!/usr/bin/python\n\n" 
    << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n\n"
    << "poses = [\n";

  CartesianPose currentPose = startPose;
  for (int i = 0; i < nEntries; i++)
  {
    Eigen::Vector6d pose = poseVelocities.col(i);
    
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
    
    Eigen::Vector3d eulerAngles = currentPose.orientation.toRotationMatrix().eulerAngles(2,1,0);

    pythonScript << currentPose.position[0] << "," << currentPose.position[1] << "," << currentPose.position[2] << "," 
      << eulerAngles[0] << "," << eulerAngles[1] << "," << std::endl;
  }
  pythonScript << "]\n";

  std::ofstream file("plot_poses.py");
  if (!file.is_open())
  {
    std::cout << "Could not write to file \"plot_poses.py\"." << std::endl;
    return;
  }
  
  file << pythonScript.str();
  file.close();
  std::cout << "wrote file plot_poses.py" << std::endl;
}

void TrajectoryPlotter::plot()
{
  
}
