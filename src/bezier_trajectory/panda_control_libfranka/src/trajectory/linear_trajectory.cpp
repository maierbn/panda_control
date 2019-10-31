#include "linear_trajectory.h"

#include <iostream>

#include "utility/smooth_motion_profile.h"

LinearTrajectory::LinearTrajectory(const CartesianPose &startPose, const CartesianPose &endPose, double vMax, double aMax, double dt) :
  startPose_(startPose), endPose_(endPose), vMax_(vMax), aMax_(aMax), dt_(dt)
{
  startPose_.orientation.normalize();
  endPose_.orientation.normalize();

  // compute length of trajectory
  length_ = std::max(startPose.getTranslationalDistance(endPose), startPose.getRotationalDistance(endPose));
}

std::vector<CartesianPose> LinearTrajectory::poses() const
{
  // this method is called at the beginning of TrajectoryIterator, the result is only used if it is pose controlled

  // get smooth motion profile for dt
  Eigen::VectorXd motionProfileValues = SmoothMotionProfile::s_t(vMax_, aMax_, length_, dt_);
  int nValues = motionProfileValues.size();

  std::vector<CartesianPose> poses(nValues);
   
  Eigen::Vector3d direction = endPose_.position - startPose_.position;

  Eigen::Array<double,1,1> dotStartEnd;
  Eigen::Array<double,1,1> halfRotAngle;

  dotStartEnd << startPose_.orientation.dot(endPose_.orientation);
  halfRotAngle = dotStartEnd.acos();
  std::cout << "Relation between rotation angle and PI is: " << 2*halfRotAngle/M_PI << std::endl;
  //double relationAngleToPI = (2*halfRotAngle.cast<double>)/M_PI;
  
  /*
  Eigen::Array<bool,1,1> compare = (2*halfRotAngle)/M_PI >= 0.5;
  Eigen::Quaterniond startPose_slerp = Eigen::Quaterniond();
  double startPoseQuatX = startPose_.orientation.z();
  double startPoseQuatY = startPose_.orientation.y();
  double startPoseQuatZ = startPose_.orientation.z();
  double startPoseQuatW = startPose_.orientation.w();

  if ( compare(0)) {
    startPose_slerp = Eigen::Quaterniond(-startPoseQuatW,-startPoseQuatX,-startPoseQuatY,-startPoseQuatZ);
  std::cout << "Slerp direction turned around: " << std::endl;

  dotStartEnd << startPose_slerp.dot(endPose_.orientation);
  halfRotAngle = dotStartEnd.acos();
  std::cout << "Relation between rotation angle and PI is now: " << 2*halfRotAngle/M_PI << std::endl;
  }else
  {
    startPose_slerp = Eigen::Quaterniond(startPoseQuatW,startPoseQuatX,startPoseQuatY,startPoseQuatZ);
  }
  */
  // set position and orientation with motion profile values
  for (int i = 0; i < nValues; i++)
  {
    poses[i].position = startPose_.position + direction*motionProfileValues[i];
    poses[i].orientation = startPose_.orientation.slerp(motionProfileValues[i], endPose_.orientation);
  }

  return poses;
}

Eigen::Matrix6dynd LinearTrajectory::poseVelocities() const
{
  // get smooth motion profile for dt
  Eigen::VectorXd motionProfileDerivatives = SmoothMotionProfile::ds_dt(vMax_, aMax_, length_, dt_);

  // initialize result matrix
  int nValues = motionProfileDerivatives.size();
  Eigen::Matrix6dynd poseVelocities(6,nValues);

  std::cout << std::endl;
  std::cout << "LinearTrajectory" << std::endl;
  std::cout << "    from: " << startPose_ << std::endl;
  std::cout << "      to: " << endPose_ << std::endl;
  std::cout << "  length: " << length_ << " m, duration: " << endTime() << " s, " << nValues << " steps." << std::endl;
  std::cout << std::endl;


  // get constant derivative of the trajectory without motion profile
  Eigen::Vector3d direction = endPose_.position - startPose_.position;

  /*
  Eigen::AngleAxisd angleAxisDiff(startPose_.orientation.conjugate()*endPose_.orientation);
  Eigen::Vector3d orientationDifference = angleAxisDiff.angle()*angleAxisDiff.axis();
  */
 
  //---------------------------------------------------------------------------------------------
  //Testing implementation of derived slerp
  /*Sources: 
    Derivative of Slerp: https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp
    Derivative of Quaternion: https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
    Log of Quaternion: https://web.mit.edu/2.998/www/QuaternionReport1.pdf page 15 
  */
  Eigen::AngleAxisd axisangleLogSlerp(endPose_.orientation*startPose_.orientation.conjugate());
  Eigen::Vector3d imaginaryLogFromSlerp = axisangleLogSlerp.angle()*axisangleLogSlerp.axis();
  Eigen::Vector3d incremetalCartesianRotation = imaginaryLogFromSlerp;
  Eigen::Vector6d trajectoryDerivative;
  trajectoryDerivative << direction, incremetalCartesianRotation;
 //---------------------------------------------------------------------------------------------
  // approach using Euler angles
  //trajectoryDerivative = startPose_.getDifferenceTo(endPose_);

  // compute the pose velocities as derivative of the linear path times the derivative of the smooth motion profile,
  // column vector * row vector = matrix where each column is for one timestep.
  
  poseVelocities = trajectoryDerivative * motionProfileDerivatives.transpose();

  //poseVelocities = trajectoryDerivative * Eigen::VectorXd::Ones(nValues).transpose();

  // output for debugging, disabled
#if 0
  Eigen::Vector3d angles = Eigen::Vector3d::Zero();
  double t = 0;

  for (int i = 0; i < nValues; i++)
  {
    angles += trajectoryDerivative.tail<3>() * dt_;
    t += dt_;
  }
  std::cout << "t: " << t << std::endl;
  std::cout << "angles: " << angles*180./M_PI << std::endl;
  

  std::cout << "trajectoryDerivative: " << trajectoryDerivative.transpose() << std::endl;
  std::cout << "startPose_:" << startPose_ << ", \nendPose_" << endPose_ << std::endl;

  std::cout << nValues*dt_ << "=" << endTime() << std::endl;

  // check motion profile
  double s = 0;
  for (int i = 0; i < nValues; i++)
  {
    s += motionProfileDerivatives[i] * dt_;
    if (i % 100 == 0)
      std::cout << "i: " << i << ", t: " << i*dt_ << ", dt: " << dt_ << ", derivative: " << motionProfileDerivatives[i] << ", s: " << s << std::endl;
  }
  std::cout << std::endl;

  // check of poseVelocities
  Eigen::Vector6d currentPose = startPose_.getVector6d();
  for (int i = 0; i < nValues; i++)
  {
    currentPose += dt_ * poseVelocities.col(i);
    if (i % 100 == 0)
      std::cout << "i: " << i << ", t: " << i*dt_ << ", dt: " << dt_ << ", derivative: " << poseVelocities.col(i).tail<3>().transpose()*180./M_PI 
      << ", pose: " << currentPose.tail<3>().transpose()*180./M_PI << std::endl;
  }
  std::cout << "endPose: " << currentPose.tail<3>().transpose()*180./M_PI << std::endl;
#endif

  return poseVelocities;
}

double LinearTrajectory::dt() const
{
  return dt_;
}

double LinearTrajectory::endTime() const
{
  return SmoothMotionProfile::endTime(vMax_, aMax_, length_, dt_);
}
