#include "utility/trajectory_iterator_cartesian.h"

#include <iostream>
#include "utility/kbhit.h"

TrajectoryIteratorCartesian::TrajectoryIteratorCartesian(Trajectory &trajectory)
    : trajectory_(trajectory), poseVelocities_(trajectory_.poseVelocities()), dt_(trajectory.dt()), endTime_(trajectory.endTime()), currentIndex_(0), currentTime_(0.0)
{

}

std::array<double, 16> TrajectoryIteratorCartesian::getCartesianPose()
{
  // if poses have not yet been retrived, get them now
  if (this->poses_.empty())
  {
    poses_ = std::vector<CartesianPose>(trajectory_.poses());
  }
  return this->poses_[this->currentIndex_].getHomogenousTransformArray();
}

std::array<double,6> TrajectoryIteratorCartesian::getCartesianVelocity()
{
  /*// if poses have not yet been retrived, get them now
  if (this->poseVelocities_.cols() == 0)
  {
    poseVelocities_ = trajectory_.poseVelocities();
  }*/
  const Eigen::Vector6d currentVelocity = this->poseVelocities_.col(this->currentIndex_);

  std::array<double,6> result;
  Eigen::Vector6d::Map(result.data()) = currentVelocity;
  return result;
}

void TrajectoryIteratorCartesian::step()
{
  currentIndex_++;
  currentTime_ += dt_;
}

double TrajectoryIteratorCartesian::currentTime() const
{
  return currentTime_; 
}

double TrajectoryIteratorCartesian::getEndTime() const
{
  return this->endTime_;
}

franka::CartesianPose TrajectoryIteratorCartesian::
operator()(const franka::RobotState &, franka::Duration)
{
  step();
  // TODO (Hinze, Maier): += time_step.toSec() führt nach x Iterationen zu Rundundgsfehlern und damit zu Sprüngen.
  //                      Der Fall, dass eine Berechnung zu lang braucht, wird schon von FrankaEmika abgefangen.
  //                      Habe es as quick fix noch mal durch `step()` ersetzt.
  //                      Wie könnten wir das sonst lösen? 
  // currentTime_ += time_step.toSec();

  // int i = 0;
  // for (; currentTime_ >= (this->currentIndex_+1) * this->dt_; i++)
  //   this->currentIndex_++;

  // only do expensive checks after 100 iterations
  if (this->currentIndex_ % 100 == 0)
  {
    // check if key was pressed, then abort
    if ( _kbhit())
    {
      std::cout << "Key was pressed, abort!" << std::endl;
      throw std::exception();
    }

  }

  if (currentTime() < getEndTime()) {
    return franka::CartesianPose(getCartesianPose());
  } else {
    return franka::MotionFinished(franka::CartesianPose(getCartesianPose()));
  }
}
