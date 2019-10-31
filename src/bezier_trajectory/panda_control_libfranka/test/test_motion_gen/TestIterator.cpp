#include "../catch.hpp"
#include "trajectory/linear_trajectory.h"
#include "utility/trajectory_iterator_cartesian.h"
#include "utility/trajectory_iterator_cartesian_velocity.h"

#include <iostream>

using namespace std::chrono_literals;

TEST_CASE("TestTrajIterator") {
  CartesianPose from = CartesianPose(Eigen::Vector3d(1,2,3), Eigen::Quaterniond::Identity());
  double maxRot = 0.2;
  Eigen::Quaterniond smallRot = Eigen::Quaterniond(maxRot, 1, 1, 1);
  smallRot.normalize();
  maxRot = smallRot.w(); // set to new w after normalization.

  CartesianPose to = CartesianPose( from.position + Eigen::Vector3d::Ones(), smallRot);

  auto traj = LinearTrajectory(from, to, /*v_max =*/1.0, /*a_max =*/5.0, /*dt =*/0.001);

  auto itr = std::make_unique<TrajectoryIteratorCartesian>(traj);

  REQUIRE(itr->getEndTime()== traj.endTime());
  REQUIRE(itr->currentTime() == Approx(0.0).margin(1e-6) );

  SECTION("Traj") {
    std::vector<CartesianPose> p = traj.poses();
    std::vector<CartesianPose> p2;

    for (const CartesianPose& elem : p) {
      p2.push_back(CartesianPose(elem.getHomogenousTransformArray()));
    }

    CAPTURE(p2.size());
    CAPTURE(p.at(0));
    // CAPTURE(p2.transpose());
    for (std::size_t i = 0; i < p.size(); ++i){
      Eigen::Vector3d e_trans = (p2.at(i).position - p.at(i).position);
      REQUIRE( e_trans.squaredNorm() <= 1e-9 );

      REQUIRE( ( p2.at(i).orientation.vec() - p.at(i).orientation.vec()).squaredNorm() <= 1e-9 );
      double e_angle = (p2.at(i).orientation.w() - p.at(i).orientation.w());
      REQUIRE( e_angle*e_angle <= 1e-9 );
    }
    
  }

  SECTION("PoseValues") {
    while (itr->currentTime() < itr->getEndTime() ) {
      CartesianPose cartPose = CartesianPose(itr->getCartesianPose());
      CAPTURE(cartPose);
      REQUIRE((cartPose.position.array() >= from.position.array()).all());
      REQUIRE((cartPose.position.array() <= to.position.array() + 1e-12).all());
      REQUIRE(cartPose.orientation.w() <= 1);
      REQUIRE(cartPose.orientation.w() >= maxRot);

      itr->step();
    }
  }

  SECTION("VelocityValues") {
    auto allZero = Eigen::Vector6d::Zero();
    auto allVMax = Eigen::Vector6d::Constant(1.0);

    Eigen::Vector6d cartVel;
    while (itr->currentTime() < itr->getEndTime()) {
      cartVel = Eigen::Vector6d(itr->getCartesianVelocity().data());
      CAPTURE(cartVel);
      REQUIRE((cartVel.array() >= allZero.array()).all());
      REQUIRE((cartVel.array() <= allVMax.array()).all());
      itr->step();
    }
  }

    SECTION("Segfault at Pose velocity iteration from main()"){
    bool endReached = false;
    Eigen::Vector6d velocities;

    franka::Duration mockDuration = franka::Duration(1ms);
    franka::RobotState mockRobotState = franka::RobotState();

    CartesianPose initialPose = CartesianPose(Eigen::Vector3d::Random(), Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX() ));
    CartesianPose targetPose = initialPose;
    targetPose.position = (targetPose.position + Eigen::Vector3d(0.2, 0.2, 0.1));
    targetPose.orientation = (targetPose.orientation * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));

    auto traj = LinearTrajectory(initialPose, targetPose, 0.05, 0.5, 1.e-3);
    auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(traj);

    do {

      auto cartesianVels = (*motionIterator)(mockRobotState, mockDuration);

      endReached =
          cartesianVels.motion_finished; // extract if motion is finished from
                                         // franka::CartesianVelocities -|> franka::Finishable
      velocities = Eigen::Vector6d(cartesianVels.O_dP_EE.data());

      CAPTURE(velocities);
      REQUIRE((velocities.head<3>().array()>=0).all());

    } while(!endReached);
  }

  SECTION("Operator() positions") {
    franka::Duration mockDuration = franka::Duration(1ms);
    franka::RobotState mockRobotState = franka::RobotState();

    std::size_t i=1;
    auto cartesianPoses = traj.poses();

    while (itr->currentTime() < itr->getEndTime()) {
      franka::CartesianPose currentPose = (*itr)(mockRobotState, mockDuration);

      CartesianPose cp = CartesianPose(currentPose.O_T_EE);
      CAPTURE(itr->currentTime() );
      CAPTURE(i);
      CAPTURE(cartesianPoses.at(i)-cp);
      CAPTURE(cp);
      // Check that currentPose contains same values as generated linear trajectory.
      REQUIRE( ((cp.position.array() - cartesianPoses.at(i).position.array() ).abs() <= 5e-4).all());

      ++i;
    }

  }
}

TEST_CASE("TestTrajIteratorVelocity") {
  CartesianPose from = CartesianPose(Eigen::Vector3d(1,2,3), Eigen::Quaterniond::Identity());
  double maxRot = 0.2;
  Eigen::Quaterniond smallRot = Eigen::Quaterniond(maxRot, 1, 1, 1);
  smallRot.normalize();
  CartesianPose to = CartesianPose( from.position+ Eigen::Vector3d::Ones(), smallRot);

  auto allZero = Eigen::Vector6d::Zero();
  auto allVMax = Eigen::Vector6d::Constant(1.0);

  auto traj = LinearTrajectory(from, to, /*v_max =*/1.0, /*a_max =*/5.0, /*dt =*/0.001);

  auto itr = std::make_unique<TrajectoryIteratorCartesianVelocity>(traj);

  SECTION("Operator() velocities") {
    bool endReached = false;
    Eigen::Vector6d velocities;

    franka::Duration mockDuration = franka::Duration(1ms);
    franka::RobotState mockRobotState = franka::RobotState();

    do {
      // function that is called in franka::Robot.control();
      franka::CartesianVelocities cartesianVels = (*itr)(mockRobotState, mockDuration);
      endReached =
          cartesianVels.motion_finished; // extract if motion is finished from
                                         // franka::CartesianVelocities -|> franka::Finishable
      velocities = Eigen::Vector6d(cartesianVels.O_dP_EE.data());

      CAPTURE(velocities);
      REQUIRE((velocities.array() >= allZero.array()).all());
      REQUIRE((velocities.array() <= allVMax.array()).all());

    } while (!endReached);
  }


}