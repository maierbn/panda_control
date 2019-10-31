#include "utility/trajectory_iterator_cartesian_velocity.h"
#include "trajectory/linear_trajectory.h"
#include "trajectory/curve_trajectory.h"
#include "trajectory/smooth_curve_trajectory.h"
#include "utility/trajectory_plotter.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

#include <iostream>

const std::string robot_ip = "172.16.0.2";

void setDefaultBehaviour(franka::Robot &robot);

const double endTime = 15.0;    // duration of the trajectory curve(t), t ∈ [0,endTime]

/** example curve function used for the trajectory */
CartesianPose curve(double t)
{
  double alpha = t / endTime;  // run from 0 to 1

  const double h = 1e-4;
  alpha = (alpha * (1.-2.*h)) + h;

  //std::cout << "t: " << t << ", alpha: " << alpha << std::endl;

  double beta = 2*alpha - 1;      // run from -1 to 1

  CartesianPose result;

  const int nPoses = 451;
  const double poses[5][nPoses] = 
  {
    {0,0.010179,0.017484,0.019839,0.022551,0.025681,0.029308,0.032706,0.036203,0.040088,0.042602,0.046178,0.050111,0.054241,0.055693,0.059474,0.063486,0.065299,0.068925,0.072724,0.076658,0.078464,0.082111,0.085908,0.088205,0.091826,0.095648,0.099655,0.10182,0.10575,0.10986,0.11411,0.11621,0.12039,0.12464,0.12888,0.1311,0.13526,0.13936,0.14348,0.14586,0.1498,0.15396,0.15635,0.1604,0.16519,0.17074,0.17191,0.17739,0.183,0.18845,0.1896,0.19478,0.1997,0.20431,0.20863,0.20983,0.2139,0.21772,0.2213,0.22466,0.2278,0.23073,0.23267,0.23435,0.23694,0.23868,0.2396,0.24373,0.2457,0.24753,0.24763,0.24882,0.25084,0.25183,0.2513,0.25255,0.25405,0.25506,0.25567,0.25483,0.25554,0.25615,0.25666,0.25529,0.25424,0.25356,0.25292,0.25278,0.25221,0.25281},
    {1.9023e-17,0.0081835,0.015182,0.017182,0.019409,0.021913,0.024681,0.027547,0.029948,0.032551,0.035876,0.038088,0.040418,0.042825,0.046096,0.04819,0.050318,0.053999,0.055921,0.057847,0.059738,0.063368,0.065158,0.066889,0.070369,0.072051,0.073635,0.075188,0.078336,0.079837,0.081253,0.082653,0.085337,0.086667,0.087869,0.089079,0.091273,0.092298,0.093433,0.094784,0.095829,0.096738,0.098075,0.098157,0.098655,0.099229,0.10001,0.099102,0.099875,0.10076,0.10177,0.098128,0.09858,0.099162,0.099832,0.10056,0.094815,0.095054,0.095383,0.095763,0.096205,0.096619,0.097063,0.097589,0.089514,0.089728,0.089995,0.08222,0.090478,0.090752,0.091026,0.082581,0.08276,0.082878,0.083083,0.0755,0.075575,0.075634,0.075733,0.075859,0.069585,0.069631,0.069683,0.069739,0.06513,0.06218,0.060493,0.059234,0.058986,0.058168,0.058964},
    {0.31067,0.30685,0.30018,0.29469,0.28999,0.2858,0.28194,0.2779,0.27413,0.27053,0.26677,0.26317,0.25967,0.25623,0.2527,0.24925,0.24584,0.24236,0.23893,0.23554,0.23217,0.22876,0.22538,0.22202,0.21862,0.21525,0.2119,0.20856,0.20519,0.20184,0.1985,0.19517,0.19183,0.18849,0.18517,0.18185,0.17851,0.17519,0.17187,0.16855,0.16523,0.16191,0.1586,0.15528,0.15196,0.14865,0.14534,0.14202,0.13871,0.1354,0.1321,0.12878,0.12548,0.12217,0.11886,0.11555,0.11224,0.10894,0.10563,0.10232,0.099019,0.095714,0.09241,0.089107,0.085802,0.082497,0.079193,0.075887,0.072584,0.069282,0.06598,0.062677,0.059374,0.056072,0.05277,0.049468,0.046166,0.042865,0.039564,0.036264,0.032964,0.029665,0.026366,0.023068,0.019771,0.016475,0.01318,0.0098864,0.0065916,0.0033052,0.0016592},
    {1.5708,1.5708,1.5708,1.5708,1.571,1.5711,1.5706,1.571,1.5697,1.5671,1.5692,1.5665,1.5646,1.5665,1.565,1.5667,1.5743,1.567,1.5735,1.5861,1.6046,1.5846,1.6008,1.621,1.5993,1.6176,1.638,1.6588,1.6341,1.653,1.6714,1.6879,1.6648,1.6798,1.6934,1.7047,1.684,1.6948,1.7024,1.7061,1.6927,1.6983,1.6981,1.6904,1.696,1.6988,1.6974,1.6966,1.6938,1.6885,1.681,1.7048,1.7024,1.6982,1.6925,1.6859,1.7315,1.7303,1.7278,1.7245,1.7204,1.7165,1.7122,1.7066,1.7825,1.7808,1.7781,1.8571,1.7739,1.7712,1.7684,1.8561,1.8543,1.8536,1.8513,1.9378,1.9373,1.9373,1.9364,1.9348,2.0144,2.014,2.0135,2.0127,2.0776,2.1241,2.1541,2.1807,2.1874,2.2257,2.1879},
    {0,0.0057298,0.0061631,0.0077201,0.0096114,0.011809,0.014819,0.012949,0.016218,0.02054,0.016252,0.020617,0.026193,0.032981,0.0244,0.030508,0.036514,0.027915,0.033143,0.037096,0.038765,0.033644,0.034919,0.034116,0.031456,0.030272,0.028004,0.025297,0.024096,0.021379,0.019201,0.018127,0.015688,0.014415,0.01411,0.014774,0.011267,0.012021,0.014113,0.018659,0.012089,0.016414,0.026687,0.015009,0.026617,0.052226,0.092787,0.059833,0.10492,0.15647,0.20975,0.17438,0.22916,0.28345,0.33639,0.38765,0.35871,0.41109,0.46201,0.51136,0.55922,0.60555,0.65057,0.68114,0.68076,0.72638,0.75821,0.75843,0.85675,0.89827,0.93889,0.93868,0.9686,1.0235,1.0513,1.0593,1.1023,1.1588,1.2007,1.2266,1.2644,1.3073,1.349,1.3892,1.4226,1.4586,1.4931,1.5191,1.5265,1.5546,1.5638}
  };

  int index = int(alpha * nPoses);
  double a = alpha * nPoses - index;

  double currentPose[5];
  for (int i = 0; i < 5; i++)
  {
    currentPose[i] = (1.-a) * poses[i][index] + a * poses[i][index+1];
  }

  // specify function in cm
  result.position[0] = currentPose[0];          // x
  result.position[1] = currentPose[1]; //-sin(M_PI*alpha)*10.0;          // y
  result.position[2] = currentPose[2];  // z

  
  result.position[0] = cos(M_PI_2*alpha)*30;          // x
  result.position[1] = -sin(M_PI_2*alpha)*10;          // y
  result.position[2] = 0;  // z

//std::cout << "inedx: " << index << ", a: " << a << ", index: " << index << ", currentPose: " << result.position;

  // transform from centi-meters to meters
  result.position *= 1e-2;

  // no angle change
  result.orientation = CartesianPose::neutralOrientation;
  
  double theta = currentPose[3];
  double phi = currentPose[4];

  Eigen::AngleAxisd angle0(M_PI/2. - theta, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd angle1(phi, Eigen::Vector3d::UnitZ());

  //result.orientation = CartesianPose::neutralOrientation * Eigen::Quaterniond(angle0) * Eigen::Quaterniond(angle1);
  //result.orientation = Eigen::Quaterniond::Identity();

  //std::cout << "curve(" << t << "): " << result << std::endl;
  return result;
}

int main()
{

  CartesianPose restingPose;
  restingPose.position <<  0.384663, -0.380291, 0.204745;  // right, above the wooden bottom plate
    

  const double samplingTimestepWidth = 1e-3;
  SmoothCurveTrajectory curveTrajectory(restingPose, curve, endTime, samplingTimestepWidth);

  TrajectoryPlotter trajectoryPlotter(restingPose, std::make_shared<SmoothCurveTrajectory>(curveTrajectory), samplingTimestepWidth);
  trajectoryPlotter.plot();


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
    restingPose.position << 0.0325709,-0.332922,0.220434;       // left, above the wooden bottom plate
    restingPose.position[2] += 0.31067;   // move to start position above bottom
   
    restingPose.orientation = CartesianPose::neutralOrientation;
    //restingPose.orientation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0); // rotated by 180deg around z axis (such that gripper can rotate ccw)
    /* 
    Eigen::AngleAxisd angle(-M_PI_2, Eigen::Vector3d::UnitZ());
    restingPose.orientation = restingPose.orientation * Eigen::Quaterniond(angle);
*/
    // LinearTrajectory and TrajectoryIteratorCartesianVelocity object creation
    LinearTrajectory linearTrajectory(initialPose, restingPose, 0.05, 0.05, 1.e-3);
    auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(linearTrajectory);
    
    // move to resting pose
    std::cout << " \aRobot will move to resting pose, press Enter.";
    std::cin.get();
    panda.control(*motionIterator, /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);
    
    // read current pose for debugging
    franka::RobotState currentState = panda.readOnce();
    CartesianPose currentPose(currentState.O_T_EE);

    std::cout << "current pose: " << currentPose << std::endl << std::endl;

    // define trajectory from resting pose along curve
    const double samplingTimestepWidth = 1e-3;
    SmoothCurveTrajectory curveTrajectory(restingPose, curve, endTime, samplingTimestepWidth);

    TrajectoryPlotter trajectoryPlotter(restingPose, std::make_shared<SmoothCurveTrajectory>(curveTrajectory), samplingTimestepWidth);
    trajectoryPlotter.plot();

    // move along trajectory 
    auto curveMotionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(curveTrajectory);
    
    std::cout << "\aRobot will move according to trajectory, press Enter." << std::endl 
      << "Afterwards, Enter aborts the movement\a";
    std::cin.ignore();

    panda.control(*curveMotionIterator,
                  /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);

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
