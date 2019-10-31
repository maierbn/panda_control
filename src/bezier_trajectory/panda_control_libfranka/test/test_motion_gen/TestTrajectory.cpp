#include "../catch.hpp"
#include "utility/smooth_motion_profile.h"
#include "utility/cartesian_pose.h"
#include "trajectory/linear_trajectory.h"

TEST_CASE("TestTrajectory") {

  SECTION("create time potences matrix") {
    auto vv = Eigen::VectorXd::LinSpaced(10, 0., 1.);
    Eigen::MatrixXd P(2, vv.size());
    P.row(0) = vv.array().pow(2);
    P.row(1) = vv.array().pow(3.);

    INFO(P);

    REQUIRE(vv(0, 0) == 0.0);
  }

  SECTION("Test end times for Motion Profile construction") {
    double v_nom = 2.1875;
    double a_nom = 7.513188404399293;
    double L_nom = 1.0;
    double dt = 1e-3;

    double t_E;

    INFO("End time == 1 for nominal values in motion profile.")
    t_E = SmoothMotionProfile::endTime(v_nom, a_nom,L_nom, dt);
    REQUIRE(t_E == Approx(1)); // verify end time == 1

    INFO("vary v_nom");
    for (double v_m = 0.25; v_m <= 16; v_m *= 2) {
      t_E = std::max(35 * L_nom / 16 / v_m, 1.);
      REQUIRE(SmoothMotionProfile::endTime( v_m , a_nom,L_nom, dt) == Approx(t_E).margin(dt));
    }

    INFO("vary a_nom");
    for (double a_m = 1; a_m <= 128; a_m *= 2) {
      t_E = std::max(1., sqrt(a_nom * L_nom / a_m));
      REQUIRE(SmoothMotionProfile::endTime(v_nom, a_m,L_nom, dt) == Approx(t_E).margin(dt));
    }

    INFO("vary L");
    for (double a_l = .1; a_l <= 10; a_l *= 2) {
      t_E = std::max(a_l, sqrt(a_l));
      REQUIRE(SmoothMotionProfile::endTime(v_nom, a_nom, a_l, dt) == Approx(t_E).margin(dt));
    }

    INFO("vary dt");
    for (double a_dt = 1e-3; a_dt <= 1; a_dt *= 2) {
      CAPTURE(a_dt);
      REQUIRE(SmoothMotionProfile::endTime(sqrt(2) * v_nom, 2 * a_nom, L_nom, a_dt) == Approx(1 / sqrt(2)).margin(a_dt));
    }
  }
  SECTION("Motion Profile Construction") {

    double dt = 1e-3;
    double v_max = 1.0;
    double a_max = 5.0;

    INFO("Estimate velocities and accelerations with finite differences");
    const Eigen::ArrayXd p_vals = SmoothMotionProfile::s_t(/*v_max=*/v_max, /*a_max=*/a_max, /*L=*/1.0, /*dt=*/dt);
    const Eigen::ArrayXd dp_ds_vals = SmoothMotionProfile::ds_dt(/*v_max=*/v_max, /*a_max=*/a_max, /*L=*/1.0, /*dt=*/dt);
    const auto N = p_vals.size();
    const Eigen::ArrayXd v_diff =
        (p_vals.segment(2, N - 2) - p_vals.segment(0, N - 2)) / (2 * dt);
    const Eigen::ArrayXd a_diff =
        (v_diff.segment(2, N - 4) - v_diff.segment(0, N - 4)) / (2 * dt);
    // CAPTURE(p_vals);
    // CAPTURE(v_diff);

    INFO("Are the dynamics limits kept?");
    REQUIRE(v_diff.abs().maxCoeff() <=v_max);
    REQUIRE(a_diff.abs().maxCoeff() <= a_max);
    REQUIRE(dp_ds_vals.abs().maxCoeff() <= v_max);

    INFO("Are difference quotients close to real derivative( abssum(e) =" +
         std::to_string((dp_ds_vals.segment(1, N - 2) - v_diff).abs().sum()) + ")?");

    CAPTURE(dp_ds_vals.segment(ceil(N / 2), 10));
    CAPTURE(v_diff.segment(ceil(N / 2) + 1, 10));

    // INFO("DEB:");
    // CAPTURE(dp_ds_vals.segment(1, N - 2) - v_diff);

    REQUIRE((dp_ds_vals.segment(1, N - 2) - v_diff).abs().sum() / N <= 1e-6);
  }

  SECTION("Converison between homogeneous TF and 6D pose.") {
    std::array<double, 16> tfMatrix;

    for (std::size_t i = 0; i < 10; i++) {
      CartesianPose pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

      CAPTURE(pose);
      tfMatrix = pose.getHomogenousTransformArray();
      CartesianPose poseReconstructed = CartesianPose(tfMatrix);
      CAPTURE(tfMatrix);
      CAPTURE(poseReconstructed);
      REQUIRE((poseReconstructed.position - pose.position).array().abs().sum() == Approx(0.0).margin(1e-6));
      REQUIRE( poseReconstructed.orientation.angularDistance(pose.orientation) == Approx(0.0).margin(1e-8));

      std::array<double, 16> tf2 = poseReconstructed.getHomogenousTransformArray();
      CAPTURE(tf2);
      for (std::size_t i = 0; i<16; i++){
        REQUIRE(tf2[i] - tfMatrix[i] == Approx(0.0).margin(1e-10));
      }
    }

    SECTION("Reliable conversion to TF matrix, even if Quaternions are not restored identically."){
      INFO("q -> TF -> -q");
      CartesianPose pose(Eigen::Vector3d::Random(), Eigen::Quaterniond(-1, 0.2, 0.4, 0.7).normalized());
      std::array<double, 16> tfMatrix = pose.getHomogenousTransformArray();
      CartesianPose poseReconstructed = CartesianPose(tfMatrix);

      INFO("Rotation in reconstructed pose is negative quaternion of original pose.");
      REQUIRE(poseReconstructed.orientation.w() + pose.orientation.w() == Approx(0.0).margin(1e-8));
      REQUIRE( (poseReconstructed.orientation.vec() + pose.orientation.vec()).norm() == Approx(0.0).margin(1e-8) );

      INFO("Then the resulting homogeneous transformations should be the same.")
      std::array<double, 16> tf2 = poseReconstructed.getHomogenousTransformArray();

      for (std::size_t i = 0; i<16; i++){
        REQUIRE(tf2[i] - tfMatrix[i] == Approx(0.0).margin(1e-10));
      }
    }

    /* no invalid angles anymore, instead, angles have been stabilized. This is tested in TestStablePoseConversion.cpp. */
  }

  SECTION("Linear trajectory") {
    CartesianPose u = CartesianPose(Eigen::Vector3d(1,2,3), Eigen::Quaterniond::Identity());
    double maxRot = 0.2;
    Eigen::Quaterniond smallRot = Eigen::Quaterniond(maxRot, 1, 1, 1);
    smallRot.normalize();
    maxRot = smallRot.w();

    CartesianPose v = CartesianPose(Eigen::Vector3d(3, 4, 5), smallRot);

    auto traj = LinearTrajectory(u, v, /*v_max =*/1.0, /*a_max =*/5.0, /*dt =*/0.001);
    //   auto traj = LinearTrajectory(Eigen::Vector6d::Zero(), Eigen::Vector6d::Ones(), 2.0
    //   /*[m/s]*/,
    //                                10.0 /*[m/s^2]*/, 1e-3 /*[s]*/);
    std::vector<CartesianPose> p = traj.poses();
    Eigen::Matrix6dynd dp_dt = traj.poseVelocities();

    REQUIRE(traj.dt() == 1e-3);

    CAPTURE(traj.dt());
    CAPTURE(p.size());
    REQUIRE(traj.endTime() == traj.dt() * (p.size()-1));

    INFO("Position is kept");
    for (const auto elem : p) {
      REQUIRE((elem.position.array() >= u.position.array()).all());
      REQUIRE((elem.position.array() <= v.position.array()).all());
      REQUIRE(elem.orientation.w() <= 1);
      REQUIRE(elem.orientation.w() >= maxRot );
    }

    const auto n = dp_dt.topRows(3).colwise().norm();
    REQUIRE(n.maxCoeff() <= 1.0);
  }
}