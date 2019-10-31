#include "utility/trajectory_iterator_cartesian_velocity.h"
#include "trajectory/linear_trajectory.h"
#include "trajectory/curve_trajectory.h"
#include "trajectory/smooth_curve_trajectory.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

const std::string robot_ip = "172.16.0.2";

void setDefaultBehaviour(franka::Robot &robot);

const double endTime = 4.0;    // duration of the trajectory curve(t), t ∈ [0,endTime]

/** example curve function used for the trajectory */
CartesianPose curve(double t)
{
  double alpha = t / endTime;  // run from 0 to 1
  // double beta = 2*alpha - 1;      // run from -1 to 1

  CartesianPose result;

  // specify function in cm
  result.position[0] = sin(M_PI*alpha) * 30;          // x
  result.position[1] = cos(-0.8*M_PI*alpha) * 10; //-sin(M_PI*alpha)*10.0;          // y
  result.position[2] = 0;  // z

  // transform from centi-meters to meters
  result.position *= 1e-2;

  // no angle change
  result.orientation = CartesianPose::neutralOrientation;
  
  // Eigen::AngleAxisd angle(-M_PI_2*1.5*alpha, Eigen::Vector3d::UnitZ());
  // result.orientation = CartesianPose::neutralOrientation * Eigen::Quaterniond(angle);
  //result.orientation = Eigen::Quaterniond::Identity();

  //std::cout << "curve(" << t << "): " << result << std::endl;
  return result;
}

int main()
{
  std::cout << "connect to robot " << std::endl;
  franka::Robot panda(robot_ip);

  try
  {
    // connect to robot
    setDefaultBehaviour(panda);

    // read current robot state
    franka::RobotState initialState = panda.readOnce();
    CartesianPose initialPose(initialState.O_T_EE);
    std::cout << "current pose: " << initialPose << std::endl;
    
    // calculate resting pose
    CartesianPose restingPose;
    //restingPose.position <<  0.317125, -0.38625, 0.367743;  // in the air
    restingPose.position <<  0.384663, -0.380291, 0.204745;  // right, above the wooden bottom plate
    restingPose.position <<  -0.0560702, -0.322303, 0.201182;  // center, above the wooden bottom plate
    restingPose.position <<  -0.0560702, -0.402303, 0.201182;  // center, above the wooden bottom plate
    //restingPose.position << -0.503348,-0.40,0.201182;       // left, above the wooden bottom plate
    restingPose.position << 0.0325709,-0.332922,0.220434;
           // left, above the wooden bottom plate
    restingPose.position << 0.0325709,-0.332922,0.55;
    //restingPose.orientation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0); // rotated by 180deg around z axis (such that gripper can rotate ccw)
    
    Eigen::AngleAxisd angle(0.9*M_PI_2, Eigen::Vector3d::UnitZ());
    restingPose.orientation = restingPose.orientation*Eigen::Quaterniond(angle) ;

    // LinearTrajectory and TrajectoryIteratorCartesianVelocity object creation
    LinearTrajectory linearTrajectory(initialPose, restingPose, 0.1, 0.1, 1.e-3);
    auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(linearTrajectory);
    
    // move to resting pose
    std::cout << " Robot will move to resting pose, press Enter.";
    std::cin.get();
//exit(0);
    panda.control(*motionIterator, /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);
    
    // read current pose for debugging
    franka::RobotState currentState = panda.readOnce();
    CartesianPose currentPose(currentState.O_T_EE);

    std::cout << "current pose: " << currentPose << std::endl << std::endl;

/*
    // define trajectory from resting pose along curve
    const double samplingTimestepWidth = 1e-3;
    SmoothCurveTrajectory curveTrajectory(restingPose, curve, endTime, samplingTimestepWidth);

    // move along trajectory
    auto curveMotionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(curveTrajectory);
    
    std::cout << "Robot will move according to trajectory, press Enter." << std::endl 
      << "Afterwards, Enter aborts the movement\a";
    std::cin.ignore();

    panda.control(*curveMotionIterator,
                  //controller_mode = 
                   franka::ControllerMode::kCartesianImpedance);
  */
  }
  catch (const franka::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  catch (const std::invalid_argument &e)
  {
    std::cout << e.what() << std::endl;
    return -2;
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -10;
  }

  panda.automaticErrorRecovery();

  std::cout << "Motion finished regularly." << std::endl;
  return 0;
}

void setDefaultBehaviour(franka::Robot &robot)
{

  robot.automaticErrorRecovery();
  const double safetyFactor = 1.0;     // seems to have no effect at all

  const double torqueContactAcceleration =    safetyFactor * 30;     //20
  const double torqueCollisionAcceleration =  safetyFactor * 30;  // 20
  const double torqueContactNominal =         safetyFactor * 30;     // 10
  const double torqueCollisionNominal =       safetyFactor * 30;  // 10
  const double forceContactAcceleration =     safetyFactor * 20;    // 20
  const double forceCollisionAcceleration =   safetyFactor * 20;  // 20
  const double forceContactNominal =          safetyFactor * 10;     // 10
  const double forceCollisionNominal =        safetyFactor * 10;   // 10

  const std::array<double, 7>	lower_torque_thresholds_acceleration({torqueContactAcceleration, torqueContactAcceleration, torqueContactAcceleration, torqueContactAcceleration, torqueContactAcceleration, torqueContactAcceleration, torqueContactAcceleration});   // contact torque
  const std::array<double, 7>	upper_torque_thresholds_acceleration{torqueCollisionAcceleration, torqueCollisionAcceleration, torqueCollisionAcceleration, torqueCollisionAcceleration, torqueCollisionAcceleration, torqueCollisionAcceleration, torqueCollisionAcceleration};   // collision torque
  const std::array<double, 7>	lower_torque_thresholds_nominal{torqueContactNominal, torqueContactNominal, torqueContactNominal, torqueContactNominal, torqueContactNominal, torqueContactNominal, torqueContactNominal};        // contact torque
  const std::array<double, 7>	upper_torque_thresholds_nominal{torqueCollisionNominal, torqueCollisionNominal, torqueCollisionNominal, torqueCollisionNominal, torqueCollisionNominal, torqueCollisionNominal, torqueCollisionNominal};        // collision torque
  const std::array<double, 6>	lower_force_thresholds_acceleration{forceContactAcceleration, forceContactAcceleration, forceContactAcceleration, forceContactAcceleration, forceContactAcceleration, forceContactAcceleration};
  const std::array<double, 6>	upper_force_thresholds_acceleration{forceCollisionAcceleration, forceCollisionAcceleration, forceCollisionAcceleration, forceCollisionAcceleration, forceCollisionAcceleration, forceCollisionAcceleration};
  const std::array<double, 6>	lower_force_thresholds_nominal{forceContactNominal, forceContactNominal, forceContactNominal, forceContactNominal, forceContactNominal, forceContactNominal};
  const std::array<double, 6>	upper_force_thresholds_nominal{forceCollisionNominal, forceCollisionNominal, forceCollisionNominal, forceCollisionNominal, forceCollisionNominal, forceCollisionNominal};

  robot.setCollisionBehavior(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal,      upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration,  upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal,       upper_force_thresholds_nominal);
  robot.automaticErrorRecovery();

  const double impedance_scaling_factor = 1.0;  // don't change
  robot.setJointImpedance({{3000*impedance_scaling_factor, 3000*impedance_scaling_factor, 3000*impedance_scaling_factor, 2500*impedance_scaling_factor, 2500*impedance_scaling_factor, 2000*impedance_scaling_factor, 2000*impedance_scaling_factor}});
  robot.setCartesianImpedance({{3000*impedance_scaling_factor, 3000*impedance_scaling_factor, 3000*impedance_scaling_factor, 300*impedance_scaling_factor, 300*impedance_scaling_factor, 300*impedance_scaling_factor}});
}
