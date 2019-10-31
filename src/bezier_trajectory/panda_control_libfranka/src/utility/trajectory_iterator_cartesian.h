#pragma once

#include "utility/cartesian_pose.h"
#include "trajectory/trajectory.h"

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <memory>
#include <array>
#include <vector>


/** \brief General iterator for cartesian paths, providing methods for obtaining pose, poseVelovity
 * and iterate one time step.
 *
 *  This can be used to manually build a position or velocity trajectory for the robot. For the
 * Franka Emika Panda it is easier to use TrajectoryIteratorCartesianVelocity.
 */
class TrajectoryIteratorCartesian { 

public:
  TrajectoryIteratorCartesian(const Trajectory &traj);

  /** \brief get current cartesian pose (x,y,z, R,P,Y).
   *
   * \returns the current cartesian pose as homogeneous transformation matrix (16 double values)
   *
   * \note the return value will be the same, until step() is called.
   *
   */
  std::array<double, 16> getCartesianPose() const;

  /** \brief get current cartesian velocities
   *
   * \returns the current velocities (v_x, v_y, v_z, omega_x, omega_x, omega_z), v in [m/s], omega
   * in [rad/s].
   *
   * \note To iterate over the dataset, call step(). Otherwise, the same value is fetched every
   * time.
   * */
  std::array<double, 6> getCartesianVelocity() const;

  /** \brief iterate to the next time instance.
   * 
   * the internal pointer is counted up, pointing to the next instance for pose and velocities.
   */
  void step();

  /** \brief get the current time [s], which is calculated from the number of steps taken and the
   * step width dt.
   *
   * \returns the current time [s].  */
  double currentTime() const;

  /** \brief get the precalculated end time of the current trajectory.
   *
   * This can be used to compare, if the tajectory is finished already, i.e.
   *
   * ```{.cpp}
   * if (currentTime() <= getEndTime()){
   *  std::cout<<"We are before the trajectory's end."<<std::endl;
   * }else{
   *  std::cout<<"End of trajectory is reached."<<std::endl;
   * }
   * ```
   * \returns calculated end time [s].
   *
   */
  double getEndTime() const;
  
  /** \brief Operator for directly using an objuect of this class within the Franka::Robot::control() function.
   * 
   * \todo (Hinze, Maier): Why are we iterating by time_step here? Isn't the whole concept made for fixed dt distances?
   * */
  franka::CartesianPose operator()(const franka::RobotState &, franka::Duration time_step);

protected:
  /// column-wise storage of precalculated poses for the whole trajectory (end effector in
  /// robot base coordinate system).
  std::vector<CartesianPose> poses_;

  /// column-wise storage of precalculated pose velocities for the whole trajectory (end effector in
  /// robot base coordinate system).
  Eigen::Matrix6dynd poseVelocities_;

  /// step size between time steps [s]
  const double dt_;

  // end time of the trajcectory. Assuming it starts at 0 and progresses constantly with dt [s]
  const double endTime_;

  /// current iteration number
  Eigen::Index currentIndex_;

  // current time of robot
  double currentTime_; 
};
