#include "curve_trajectory.h"

#include <iomanip>
#include <iostream>
#include <fstream>

CurveTrajectory::CurveTrajectory(const CartesianPose initialPose,
  std::function<CartesianPose (double t)> curve, double endTime, double dt) :
  endTime_(endTime), dt_(dt), initialPose_(initialPose)
{
  // compute dt such that it allows constant time step widths within endTime
  int nSteps = int(round(endTime / dt));
  dt_ = endTime / nSteps;

  curve_ = curve;
  curveStartPose_ = curve_(0);
}

std::vector<CartesianPose> CurveTrajectory::poses() const {
  
  int nSteps = int(round(endTime_ / dt_));

  std::vector<CartesianPose> poses(nSteps);

  for (int i = 0; i < nSteps; i++)
  {
    // compute current time
    double t = i*dt_;

    poses[i] = curve_(t) - curveStartPose_ + initialPose_;
  }

  // TODO(Benni): dump Poses -> output
  // compute 2D surface coordinates and angle
  #if 0
  Eigen::Matrix<double, Eigen::Dynamic, 9> positions(nSteps,9);  // [x,y,angle, x',y',angle', x'',y'',angle'']
  for (int i = 0; i < nSteps; i++)
  {
    double x = poses(0,i);  // x
    double y = poses(1,i);  // y

    Eigen::Quaterniond orientation(poses(6,i), poses(3,i), poses(4,i), poses(5,i));
    double angle = orientation.toRotationMatrix().eulerAngles(0,1,2)[2];  // angle around z-axis

    positions(i,0) = x;
    positions(i,1) = y;
    positions(i,2) = angle;

    //std::cout << "x,y,angle: " << x << ", " << y  << ", " << angle * 180./M_PI << std::endl;
  }

  // compute first derivatives
  for (int i = 0; i < nSteps; i++)
  {
    int iplus = std::min(i+1,nSteps-1);
    int iminus = std::max(i-1,0);

    for (int j = 0; j < 3; j++)
    {
      positions(i,j+3) = (positions(iplus,j) - positions(iminus,j)) / ((iplus - iminus)*dt_);
    }
  }
  
  // compute second derivatives
  for (int i = 0; i < nSteps; i++)
  {
    int iplus = std::min(i+1,nSteps-1);
    int iminus = std::max(i-1,0);

    for (int j = 3; j < 6; j++)
    {
      positions(i,j+3) = (positions(iplus,j) - positions(iminus,j)) / ((iplus - iminus)*dt_);
    }
  }

  // write positions to file
  std::string filename = "tube5.csv";
  std::ofstream file(filename, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open())
  {
    std::cout << "Could not open file \"" << filename << "\" for writing." << std::endl;
    exit(1);
  }
   
  for (int i = 0; i < nSteps; i++)
  {
    file << i*dt_;
    for (int j = 0; j < 9; j++)
    {
      file << ",";
      file << positions(i,j);
    }
    file << std::endl;
  }

  file.close();
  std::cout << "Wrote x,y,phi to file \"" << filename << "\"." << std::endl;
  #endif

  return poses;
}

Eigen::Matrix6dynd CurveTrajectory::poseVelocities() const
{
  Eigen::Vector6d velocity;

  int nSteps = int(round(endTime_ / dt_));

  CartesianPose endPose = curve_(endTime_) - curveStartPose_ + initialPose_;

  std::cout << std::endl;
  std::cout << "CurveTrajectory" << std::endl;
  std::cout << "    from: " << initialPose_ << std::endl;
  std::cout << "      to: " << endPose << std::endl;
  std::cout << "  dt: " << dt_ << " m, duration: " << endTime_ << " s, " << nSteps << " steps." << std::endl;
  std::cout << std::endl;

  Eigen::Matrix6dynd velocities(6,nSteps);

  const double h = 1e-5;   // do not set too small!

  for (int i = 0; i < nSteps; i++)
  {
    // compute current time
    double t = i*dt_;

    //std::cout << "t: " << t << ", curve(" << t << "): " << curve_(t) << std::endl;

    // compute velocity by 2nd order central difference quotient
    CartesianPose curve0 = curve_(t-h);
    CartesianPose curve1 = curve_(t+h);

    // compute translational and rotational velocity
    Eigen::Vector6d velocity = curve0.getDifferenceTo(curve1) / (2*h);

    velocities.col(i) = velocity;

    //std::cout << "  v: " << velocities.col(i).transpose() << std::endl;
  }
  return velocities;

}

double CurveTrajectory::dt() const {
  return dt_;
}

double CurveTrajectory::endTime() const {
  return endTime_;
}
