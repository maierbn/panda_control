#include "utility/trajectory_iterator_cartesian_velocity.h"

#include <iostream>
#include "utility/kbhit.h"

franka::CartesianVelocities TrajectoryIteratorCartesianVelocity::
operator()(const franka::RobotState &, franka::Duration time_step)
{
  franka::CartesianVelocities cartesianVelDes = franka::CartesianVelocities(getCartesianVelocity());
  
  step();
  // TODO (Hinze, Maier): += time_step.toSec() führt nach x Iterationen zu Rundundgsfehlern und damit zu Sprüngen.
  //                      Wie könnten wir das sonst lösen? 
  currentTime_ += time_step.toSec();

  int i = 0;
  for (; currentTime_ >= (this->currentIndex_+1) * this->dt_; i++)
    this->currentIndex_++;

  // only do expensive checks after 100 iterations
  if (this->currentIndex_ % 100 == 0)
  {
    // check if key was pressed, then abort
    if ( _kbhit())
    {
      std::cout << "Key was pressed, abort!" << std::endl;
      throw std::runtime_error("Abort key pressed");
    }

    // check if proceeded further than one sample
    if (i > 1)
      std::cout << "Warning: Sampling width is too small, jumping to " << i << "th sample!" << std::endl;
  }

  if (currentTime() < getEndTime()) {
    if (false)
    {
      std::cout << "t: " << currentTime_ << ", cartVelocity: " 
        << "  vx: " << cartesianVelDes.O_dP_EE[0] << "," 
        << "  vy: " << cartesianVelDes.O_dP_EE[1] << "," 
        << "  vz: " << cartesianVelDes.O_dP_EE[2] << "," 
        << "  omegax: " << cartesianVelDes.O_dP_EE[3] << "," 
        << "  omegay: " << cartesianVelDes.O_dP_EE[4] << "," 
        << "  omegaz: " << cartesianVelDes.O_dP_EE[5] << std::endl;
    }
    return cartesianVelDes;
  } else {
    return franka::MotionFinished(cartesianVelDes);
  }
}
