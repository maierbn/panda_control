#include "utility/trajectory_iterator_cartesian_velocity.h"
#include "trajectory/linear_trajectory.h"
#include "trajectory/curve_trajectory.h"
#include "trajectory/smooth_curve_trajectory.h"
#include "trajectory/bezier_trajectory.h"
#include "utility/trajectory_plotter.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <fstream>
#include <array>
#include <vector>
#include <iostream>

#include <iostream>

//#define NO_ROBOT

const std::string robot_ip = "172.16.0.2";

void setDefaultBehaviour(franka::Robot &robot);

// global vector of poses, to be used in the curve function
std::vector<std::array<double,5>> poses;

const double endTime = 10.0;    // duration of the trajectory curve(t), t ∈ [0,endTime]

/** example curve function used for the trajectory */
CartesianPose curve(double t)
{
  double alpha = t / endTime;  // run from 0 to 1

  const double h = 1e-4;
  alpha = (alpha * (1.-2.*h)) + h;

  //std::cout << "t: " << t << ", alpha: " << alpha << std::endl;

  double beta = 2*alpha - 1;      // run from -1 to 1

  // determine current pose index for the given alpha (time) value
  const int nPoses = poses.size();
  int index = int(alpha * nPoses);

  // clamp index to range of poses
  index = std::min(nPoses-2, std::max(0, index));

  // linearly interpolate between neighbouring poses
  double lambda = alpha * nPoses - index;

  double currentPose[5];
  for (int i = 0; i < 5; i++)
  {
    currentPose[i] = (1.-lambda) * poses[index][i] + lambda * poses[index+1][i];
  }

  // specify function in cm
  CartesianPose result;
  result.position[0] = currentPose[0];          // x
  result.position[1] = currentPose[1]; //-sin(M_PI*alpha)*10.0;          // y
  result.position[2] = currentPose[2];  // z

  /*
  result.position[0] = cos(M_PI_2*alpha)*30;          // x
  result.position[1] = -sin(M_PI_2*alpha)*10;          // y
  result.position[2] = 0;  // z
*/
//std::cout << "inedx: " << index << ", a: " << a << ", index: " << index << ", currentPose: " << result.position;

  // transform from centi-meters to meters
  //result.position *= 1e-2;

  // no angle change
  result.orientation = CartesianPose::neutralOrientation;
  
  double theta = currentPose[3];
  double phi = currentPose[4];

  Eigen::AngleAxisd angle0(phi, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd angle1(M_PI/2. - theta, Eigen::Vector3d::UnitY());

  result.orientation = Eigen::Quaterniond(angle1) * Eigen::Quaterniond(angle0) * CartesianPose::neutralOrientation;
  //result.orientation = Eigen::Quaterniond::Identity();

  //std::cout << "curve(" << t << "): " << result << std::endl;
  return result;
}

int main()
{

  // load trajectory from "savedTrajectory.csv"
  std::string filename = "savedTrajectory2.csv";
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cout << "Could not open file \"" << filename << "\"." << std::endl;
    exit(-1);
  }

  // loop over lines of file
  for (int lineNo = 0;; lineNo++)
  {
    // read one line from the file
    std::string line;
    std::getline(file, line);

    if (file.eof())
    {
      break;
    }

    std::stringstream lineStream(line);

    // split line at ","
    std::string valueString;
    for (int entryNo = 0; std::getline(lineStream, valueString, ','); entryNo++)
    {
      // only use every 10th pose
      //if ((entryNo % 10) != 0)
      //  continue;

      // allocate memory if necessary
      if (entryNo >= (int)poses.size())
      {
        poses.resize(entryNo+1);
      }
    
      // convert string to double and add entry
      poses[entryNo][lineNo] = (double)atof(valueString.c_str());
    }
  }

  std::cout << "Parsed " << poses.size() << " poses from file \"" << filename << "\". " << std::endl;

  // remove some poses
  const int stride = 5;
  std::vector<std::array<double,5>> newPoses;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    if (i % stride == 0)
    {
      newPoses.push_back(poses[i]);
    }
  }

  poses.assign(newPoses.begin(), newPoses.end());

  //restingPose.position <<  0.384663, -0.380291, 0.204745;  // right, above the wooden bottom plate
    
#if 0
  // create poses in Cartesian pose
  std::vector<CartesianPose> cartesianPoses(poses.size());

  for (unsigned int i = 0; i < poses.size(); i++)
  {
    cartesianPoses[i].position[0] = poses[i][0];
    cartesianPoses[i].position[1] = poses[i][1];
    cartesianPoses[i].position[2] = poses[i][2];
      
    double theta = poses[i][3];
    double phi = poses[i][4];
      
    Eigen::AngleAxisd angle0(phi, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd angle1(M_PI/2. - theta, Eigen::Vector3d::UnitY());

    cartesianPoses[i].orientation = Eigen::Quaterniond(angle1) * Eigen::Quaterniond(angle0) * CartesianPose::neutralOrientation;
  }


  typedef BezierTrajectory TrajectoryType;

  const double p = 4;
  const double continuity = 3;
  const double samplingTimestepWidth = 1e-3;
  TrajectoryType curveTrajectory(restingPose, cartesianPoses, p, continuity, endTime, samplingTimestepWidth);

  std::cout << "curve starts at " << curve(0) << std::endl;
  std::cout << "start trajectoryPlotter" << std::endl;

  TrajectoryPlotter trajectoryPlotter(restingPose, std::make_shared<TrajectoryType>(curveTrajectory), samplingTimestepWidth);
  trajectoryPlotter.plot();

  exit(0);
#endif

  try
  {

    CartesianPose restingPose;
    {
#ifndef NO_ROBOT
      std::cout << "connect to robot " << std::endl;
      franka::Robot panda(robot_ip);

      // connect to robot
      setDefaultBehaviour(panda);

      // read current robot state
      franka::RobotState initialState = panda.readOnce();
      CartesianPose initialPose(initialState.O_T_EE);
      std::cout << "current pose: " << initialPose << std::endl;
      
#else
      std::cout << "compiled with NO_ROBOT " << std::endl;
#endif
      // calculate resting pose
      //restingPose.position <<  0.317125, -0.38625, 0.367743;  // in the air
      restingPose.position <<  0.384663, -0.380291, 0.204745;  // right, above the wooden bottom plate
      restingPose.position <<  -0.0560702, -0.322303, 0.201182;  // center, above the wooden bottom plate
      restingPose.position <<  -0.0560702, -0.402303, 0.201182;  // center, above the wooden bottom plate
      //restingPose.position << -0.503348,-0.40,0.201182;       // left, above the wooden bottom plate
      restingPose.position << 0.0325709,-0.332922,0.220434;       // left, above the wooden bottom plate
      restingPose.position[2] += 0.40;   // move to start position above bottom
    
      
      restingPose.position << -0.0201281,-0.383884,0.597121;    // center, far above plate

      restingPose.orientation = CartesianPose::neutralOrientation;
      //restingPose.orientation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0); // rotated by 180deg around z axis (such that gripper can rotate ccw)
      /* 
      Eigen::AngleAxisd angle(-M_PI_2, Eigen::Vector3d::UnitZ());
      restingPose.orientation = restingPose.orientation * Eigen::Quaterniond(angle);
*/
  #ifndef NO_ROBOT
      // LinearTrajectory and TrajectoryIteratorCartesianVelocity object creation
      LinearTrajectory linearTrajectory(initialPose, restingPose, 0.2, 0.2, 1.e-3);
      auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(linearTrajectory);
      
      // move to resting pose
      std::cout << " \aRobot will move to resting pose, press Enter.";
      std::cin.get();
      panda.control(*motionIterator, /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);

      // read current pose for debugging
      franka::RobotState currentState = panda.readOnce();
      CartesianPose currentPose(currentState.O_T_EE);

      std::cout << "current pose: " << currentPose << std::endl << std::endl;
  #endif
    }
    // define trajectory from resting pose along curve

#ifdef NO_ROBOT
    const double samplingTimestepWidth = 1e-1;
#else
    const double samplingTimestepWidth = 1e-3;
#endif
    std::cout << "samplingTimestepWidth: " << samplingTimestepWidth << std::endl;

    // define Bezier trajectory
    // setup poses
    std::vector<CartesianPose> cartesianPoses(poses.size());

    for (unsigned int i = 0; i < poses.size(); i++)
    {
      // transform from poses array (x,y,z,phi,theta) to Cartesian Pose
      cartesianPoses[i].position[0] = poses[i][0];
      cartesianPoses[i].position[1] = poses[i][1];
      cartesianPoses[i].position[2] = poses[i][2];

      // transform from centi-meters to meters
      //cartesianPoses[i].position *= 1e-2;

      // no angle change
      cartesianPoses[i].orientation = CartesianPose::neutralOrientation;
      
      double theta = poses[i][3];
      double phi = poses[i][4];

      Eigen::AngleAxisd angle0(phi, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd angle1(M_PI/2. - theta, Eigen::Vector3d::UnitY());

      cartesianPoses[i].orientation = Eigen::Quaterniond(angle1) * Eigen::Quaterniond(angle0) * CartesianPose::neutralOrientation;
    }

    int p = 5;
    int continuity = 3;
    // multiplicity = p - continuity
    BezierTrajectory curveTrajectory(restingPose, cartesianPoses, p, continuity, endTime, samplingTimestepWidth);

    TrajectoryPlotter trajectoryPlotter(restingPose, std::make_shared<BezierTrajectory>(curveTrajectory), cartesianPoses, samplingTimestepWidth);
    // move along trajectory 
    auto curveMotionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(curveTrajectory);
    
    std::cout << "\aRobot will move according to trajectory, press Enter." << std::endl 
      << "Afterwards, Enter aborts the movement\a";
    std::cin.ignore();

#ifndef NO_ROBOT
    franka::Robot panda2(robot_ip);

    panda2.control(*curveMotionIterator,
                  /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);
#endif

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

#ifndef NO_ROBOT
  //panda.automaticErrorRecovery();
#endif

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
