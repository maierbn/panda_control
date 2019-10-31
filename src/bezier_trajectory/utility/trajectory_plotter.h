#pragma once

#include "utility/cartesian_pose.h"
#include "trajectory/trajectory.h"

class TrajectoryPlotter
{
public:
  TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth = 1.e-3);

  //! create a 3D plot by calling python
  void plot();
};