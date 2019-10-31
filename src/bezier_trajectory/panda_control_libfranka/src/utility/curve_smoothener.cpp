#include "utility/curve_smoothener.h"

#include <iostream>
#include "utility/smooth_motion_profile.h"

CurveSmoothener::CurveSmoothener(std::function<CartesianPose(double)> curve, double endTime) :
  originalCurve_(curve), endTime_(endTime)
{
  // compute curve length of original curve
  originalCurveLength_ = computeArclengthOriginalCurve(endTime_);
  std::cout << "CurveSmoothener: computed arc length of original curve: " << originalCurveLength_ << std::endl;

  if (originalCurveLength_ > 1e-5)
  {

    // create the equalized curve
    equalizedCurve_ = [this](double t) -> CartesianPose
    {
      double arclength = t / this->endTime_ * this->originalCurveLength_;

      // find parameter for original_curve that gives arclength
      // find originalT such that len(originalCurve(originalT)) = arclength, originalT ∈ [0,endTime_]
      // Newton: f(x) = b  =>  x_i+1 = x_i - (f(x_i) - b) / f'(x_i)
      double b = arclength;
      const double epsilon = 1e-5;
      double originalT = 0.5*endTime_;  // initial value is center
      double f = this->computeArclengthOriginalCurve(originalT);

      for (int i = 0; i < 10 && fabs(f - b) > 1e-9; i++)
      {
        // estimate derivative by finite differences
        double leftValue = originalT - epsilon;
        if (leftValue < 0)
          leftValue = 0;

        double rightValue = originalT + epsilon;
        if (rightValue > endTime_)
          rightValue = endTime_;

        double fprime = (this->computeArclengthOriginalCurve(rightValue) - this->computeArclengthOriginalCurve(leftValue))
          / (rightValue - leftValue);

        // Newton scheme update
        originalT -= (f - b) / fprime;

        // adjust variable to bounds
        if (originalT < 0)
          originalT = 0;
        else if (originalT > endTime_)
          originalT = endTime_;

        // compute new value, needed for abortion criterion
        f = this->computeArclengthOriginalCurve(originalT);

        //std::cout << "i = " << i << ", t=" << originalT << " left/right: " << leftValue << "/" << rightValue << ", f: " << f << ", b: " << b << ", error: " << (f - b) << std::endl;
      }

      // evaluate curve
      return originalCurve_(originalT);
    };

  }
  else 
  {
    // use the normal curve
    equalizedCurve_ = [this](double t) -> CartesianPose
    {
      return originalCurve_(t);
    };
  }

  // create the smooth curve

  // compute equalized_curve(motion_profile(t))
  smoothCurve_ = [this](double t) -> CartesianPose
  {
    double motionProfile = SmoothMotionProfile::evaluate(t / endTime_);
    //double motionProfile2 = SmoothMotionProfile::evaluate(motionProfile);
    //std::cout << "t: " << t << ", t/endTime_: " << t / endTime_ << ", motionProfile: " << motionProfile << std::endl;

    return this->equalizedCurve_(motionProfile*endTime_);
  };

}

double CurveSmoothener::computeArclengthOriginalCurve(double time)
{
  const int nSamples = 100;
  Eigen::Vector3d previousPosition = originalCurve_(0).position;
  double curveLength = 0;

  for (int i = 1; i < nSamples; i++)
  {
    double t = time * i / (nSamples-1);
    Eigen::Vector3d currentPosition = originalCurve_(t).position;

    double length = (currentPosition - previousPosition).norm();
    curveLength += length;

    previousPosition = currentPosition;
  }

  return curveLength;
}

std::function<CartesianPose(double)> CurveSmoothener::equalizedCurve()
{
  return equalizedCurve_;
}

std::function<CartesianPose(double t)> CurveSmoothener::smoothCurve()
{ 
  return smoothCurve_;
}
