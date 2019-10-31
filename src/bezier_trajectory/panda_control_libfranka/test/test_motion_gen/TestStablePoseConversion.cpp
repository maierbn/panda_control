#include "../catch.hpp"


#include "trajectory/linear_trajectory.h"

#include <iostream>
#include <memory>

void isVelocityLimitKept( const Eigen::Vector3d& vel, double v_max){
  REQUIRE(vel.norm()<=v_max);
}

/** \brief We hurt the usual conversion limits to verify, that the library handles those cases.
 *
 * Conversion limits are: roll_x \in [0,π], pitch_y \in [-π, π], yaw_z \in [-π, π].
 * */
TEST_CASE("TestStablePoseConversionTrajectory") {

  double v_max = 1.0;
  double a_max = 5.0;
  double dt = 1e-2;

  SECTION("Just Translation") {

    const auto rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY()));
    const auto fromTranslation = Eigen::Vector3d(-1, -2.3, 77);
    const auto toTranslation = Eigen::Vector3d(1 / 3, 0.001, 77);

    auto startPose = CartesianPose(fromTranslation, rotation);
    auto targetPose = CartesianPose(toTranslation, rotation);

    auto lt = std::make_unique<LinearTrajectory>(startPose, targetPose, v_max, a_max, dt);

    std::vector<CartesianPose> poseTraj = lt->poses();
    Eigen::Matrix6dynd velTraj = lt->poseVelocities();

    REQUIRE((Eigen::Index)poseTraj.size() == velTraj.cols());
    REQUIRE(lt->endTime() > 0.0);

    for (std::size_t i = 0; i<poseTraj.size(); ++i){
      CartesianPose pos_i = poseTraj.at(i);
      Eigen::Vector6d vel_i = velTraj.col(i);

      REQUIRE(pos_i.getRotationalDistance(targetPose) == Approx(0.0).margin(1e-6));
      REQUIRE( (pos_i.position.array() >= fromTranslation.array() ).all() );
      REQUIRE( (pos_i.position.array() <= toTranslation.array()).all() );
      REQUIRE( (vel_i.head<3>().array() >= 0).all());
      isVelocityLimitKept(vel_i.head<3>(), v_max);
      REQUIRE( vel_i.tail<3>().norm() == Approx(0.0) );
    }
  }

  SECTION("Just Rotation") {

    const auto startRotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
    const auto endRotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d translation = Eigen::Vector3d::Random();


    auto startPose = CartesianPose(translation, startRotation);
    auto targetPose = CartesianPose(translation, endRotation);

    auto lt = std::make_unique<LinearTrajectory>(startPose, targetPose, v_max, a_max, dt);

    std::vector<CartesianPose> poseTraj = lt->poses();
    Eigen::Matrix6dynd velTraj = lt->poseVelocities();

    REQUIRE((Eigen::Index)poseTraj.size() == velTraj.cols());
    REQUIRE(lt->endTime() > 0.0);

    for (std::size_t i = 0; i<poseTraj.size(); ++i){
      CartesianPose pos_i = poseTraj.at(i);
      Eigen::AngleAxisd rot_i = Eigen::AngleAxisd(pos_i.orientation);
      Eigen::Vector6d vel_i = velTraj.col(i);

      REQUIRE(pos_i.getTranslationalDistance(targetPose) == Approx(0.0));

      REQUIRE( (rot_i.axis() - Eigen::Vector3d::UnitZ()).norm() == Approx(0.0) );
      REQUIRE( rot_i.angle() <= M_PI );
      REQUIRE( rot_i.angle() >= M_PI_4 );

      REQUIRE( vel_i.head<5>().norm() == Approx(0.0) );
      REQUIRE(vel_i[5] >= 0); // Positive rotation around z, so only velocity around 3rd rot component
      isVelocityLimitKept(vel_i.tail<3>(), v_max);
      
    }
  }

  const auto startRotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
    const auto endRotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    const Eigen::Vector3d translation = Eigen::Vector3d::Random();

    auto startPose = CartesianPose(translation, startRotation);
    auto targetPose = CartesianPose(translation, endRotation);

    auto lt = std::make_unique<LinearTrajectory>(startPose, targetPose, v_max, a_max, dt);

    std::vector<CartesianPose> poseTraj = lt->poses();
    Eigen::Matrix6dynd velTraj = lt->poseVelocities();

    REQUIRE((Eigen::Index)poseTraj.size() == velTraj.cols());
    REQUIRE(lt->endTime() >0.0);

}


TEST_CASE("Trajectory crossing pi"){
  Eigen::AngleAxisd a1(90*M_PI/180, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd a2(270*M_PI/180, Eigen::Vector3d::UnitX());

  Eigen::Vector3d commonPos = Eigen::Vector3d::Random();
  CartesianPose p1 = CartesianPose(commonPos, a1);
  CartesianPose p2 = CartesianPose(commonPos, a2);

  SECTION("Verify that trajectory goes as expected"){
    LinearTrajectory lt = LinearTrajectory(p1, p2, 10, 10, 1e-2);

    std::vector<CartesianPose> poses = lt.poses();
    Eigen::Matrix6dynd vels = lt.poseVelocities();

    REQUIRE(lt.endTime() > 0);
    REQUIRE((Eigen::Index)poses.size() == vels.cols());

    Eigen::AngleAxisd lastOrientation = Eigen::AngleAxisd(89*M_PI/180, Eigen::Vector3d::UnitX());

    for (const auto& pose : poses){
      
      Eigen::AngleAxisd currentOrientation = Eigen::AngleAxisd(pose.orientation);
      // std::cout<<"pose = {"<<pose<<"}"<<std::endl;
      // std::cout<<"currentOrientation = {"<<currentOrientation.angle()<<", "<<currentOrientation.axis().transpose()<<"}"<<std::endl;

      // The quaternion slerp algorithm will keep the rotation axis stable, until multiples of pi are passed, where the
      // whole quaternion is negated (representing the same rotation).
      // In this case the rotation axis is unitX or -unitX, depending on the angle being  <=pi or >pi.
      // Further, the angle never goes above pi or below 0, but is mirrored on the y coordinate instead.
      REQUIRE( (currentOrientation.axis().cwiseAbs() - Eigen::Vector3d::UnitX()).norm() == Approx(0).margin(1e-8) );
      REQUIRE( currentOrientation.angle() <= M_PI );
      REQUIRE ( currentOrientation.angle() >= M_PI_2 );

      lastOrientation = currentOrientation;
    }

    for (Eigen::Index i=0; i<vels.cols(); i++){
      auto v_i  = vels.col(i);
      INFO("No translations");
      REQUIRE(v_i.head<3>().sum() == Approx(0.0));

      INFO("No rotations in y or z");
      REQUIRE(v_i.tail<2>().sum() == Approx(0.0));

      INFO("Positive rotation around x");
      REQUIRE( v_i[3]>=0);
    }
  }

  
}