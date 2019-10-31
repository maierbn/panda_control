#include "peg_in_hole_trajectory.h"

#include <iostream>

PegInHoleTrajectory::PegInHoleTrajectory(Eigen::Vector3d initialPosition, double initialAngle, Eigen::Vector2d targetPoint, Eigen::Vector3d controlPoint,
                                         double endTime, double dt) :
  SmoothCurveTrajectory(CartesianPose(initialPosition, initialOrientation(initialAngle)),
    [initialPosition, initialAngle, targetPoint, controlPoint, endTime](double time) -> CartesianPose
    {
      double t = time/endTime;

      // determine control points for translation
      Eigen::Vector3d controlPoint0 = initialPosition;

      Eigen::Vector3d controlPoint1;
      controlPoint1[0] = initialPosition[0];   // x
      controlPoint1[1] = initialPosition[1] - controlPoint[0];   // y
      controlPoint1[2] = initialPosition[2] + controlPoint[1];   // z

      Eigen::Vector3d controlPoint2;
      controlPoint2[0] = initialPosition[0];   // x
      controlPoint2[1] = initialPosition[1] - targetPoint[0];   // y
      controlPoint2[2] = initialPosition[2] + targetPoint[1];   // z

      // compute translation
      CartesianPose result;
      result.position = controlPoint0 * bernstein(0,t) + controlPoint1 * bernstein(1,t) + controlPoint2 * bernstein(2,t);

      // compute orientation
      double angle0 = initialAngle;
      double angle1 = controlPoint[2];
      double angle2 = 0;

      double angle = angle0 * bernstein(0,t) + angle1 * bernstein(1,t) + angle2 * bernstein(2,t);

      // linear trajectory for debugging
      //result.position = initialPosition + t * (-initialPosition+controlPoint2);
      
      Eigen::AngleAxisd angleAxis(angle, -Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q(angleAxis);

      result.orientation = q * PegInHoleTrajectory::rotateHorizontal();
      result.orientation.normalize();
   
      //std::cout << "endTime: " << endTime << ", t: " << t << ", " << result << std::endl;

      return result;
    },
    endTime, dt
  ), initialAngle_(initialAngle)
{
}

Eigen::Quaterniond PegInHoleTrajectory::rotateHorizontal()
{
  // TODO: try out
  Eigen::AngleAxisd angle(M_PI_2, -Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(angle) * CartesianPose::neutralOrientation;
}

Eigen::Quaterniond PegInHoleTrajectory::initialOrientation(double initialAngle)
{
  Eigen::Quaterniond initialOrientation;
  Eigen::AngleAxisd angleAxis(initialAngle, -Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q(angleAxis);

  initialOrientation = q * PegInHoleTrajectory::rotateHorizontal();
  initialOrientation.normalize();

  return initialOrientation;
}

/**
  * factorial function limited for 0, 1 and 2 for internal use in bernstein
  * polynomial calculations
  *
  * @param i the integer that should be fac'd
  * @return i! or 0 if input out of range
  */
double PegInHoleTrajectory::fac(int i)
{
  switch (i) {
  case 0:
    return 1;
  case 1:
    return 1;
  case 2:
    return 2;
  default:
    return 0;
  }
}

/**
  * solves the bernstein polynomial for hardcoded degree n=2
  *
  * @param i range 0..2
  * @param t position on curve, 0..1
  * @return (n over i) * (1-t)^n-i * t^i
  */
double PegInHoleTrajectory::bernstein(int i, double t)
{
  return (fac(2) / (fac(i) * fac(2 - i))) * std::pow(1 - t, 2 - i) * std::pow(t, i);
}
