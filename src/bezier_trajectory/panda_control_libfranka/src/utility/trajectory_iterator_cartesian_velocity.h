#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <memory>
#include <array>

#include "trajectory/trajectory.h"
#include "utility/trajectory_iterator_cartesian.h"

/** \brief defines the controlller callback for speed control directly with `operator()`.
 *
 * This makes it possible to directly call
 *
 * ```{.cpp}
 * franka::Robot robot = ... ;
 * TrajectoryIteratorCartesianVelocity vObj = ...;
 * robot.contol(vObj);
 * ```
 *
 * */
class TrajectoryIteratorCartesianVelocity : public TrajectoryIteratorCartesian {
public:
  /** \brief Build a new TrajectoryIteratorCartesianVelocity for using it with `libfranka`.
   *
   * The desired velocities are extracted from the given `Trajectory`.
   *
   * \arg[in] traj The trajectory object, describing the cartesian toolpath.
   */
  TrajectoryIteratorCartesianVelocity(Trajectory &traj) : TrajectoryIteratorCartesian(traj) {}

  /** \brief function call interface `(const franka::RobotState&, franka::Duration) ->
   * franka::CartesianVelocities`, which can be directly used as velocities trajectory in
   * `libfranka`'s control (matching the velocity interface).
   *
   * The internal time pointer is advanced in each call to this function.
   *
   * \note The trajectory is passed offline, so neither the RobotState, nor the Duration
   * is used.
   *
   * \returns franka::CartesianVelocities (3 translational and 3 orientational velocities) for each
   * time step.
   *
   */
  franka::CartesianVelocities operator()(const franka::RobotState &, franka::Duration);
};
