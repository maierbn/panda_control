#include "bezier_trajectory.h"

#include <iostream>

BezierTrajectory::BezierTrajectory(CartesianPose initialPose, std::vector<CartesianPose> &poses, double p, double continuity, double endTime, double dt) :
   initialPose_(initialPose), poses_(poses), p_(p), targetMultiplicity_(p - continuity)
{
  generateKnotVector();

  smoothCurveTrajectory_ = std::make_shared<SmoothCurveTrajectory>(
    initialPose_, 
    [initialPose, poses, endTime, this](double time) -> CartesianPose
    {
      double t = time/endTime;

      CartesianPose result;

      // linear interpolation
#if 0
      double alpha = t;  // run from 0 to 1

      // determine current pose index for the given alpha (time) value
      const int nPoses = poses.size();
      int index = int(alpha * nPoses);

      // clamp index to range of poses
      index = std::min(nPoses-2, std::max(0, index));

      // linearly interpolate between neighbouring poses
      double lambda = alpha * nPoses - index;

      // interpolate position
      for (int i = 0; i < 3; i++)
      {
        result.position[i] = (1.-lambda) * poses[index].position[i] + lambda * poses[index+1].position[i];
      }

      // interpolate orientation
      for (int i = 0; i < 4; i++)
      {
        result.orientation.coeffs()[i] = (1.-lambda) * poses[index].orientation.coeffs()[i] + lambda * poses[index+1].orientation.coeffs()[i];
      }
      result.orientation.normalize();
#endif
      // create bezier curve
      result.position = Eigen::Vector3d::Zero();
      result.orientation.coeffs() = Eigen::Vector4d::Zero();
      for (int i = 0; i < this->nBasisFunctions_; i++)
      {
        result.position += poses_[i].position * basis(i,p_,t);
        result.orientation.coeffs() += poses_[i].orientation.coeffs() * basis(i,p_,t);
      }
      result.orientation.normalize();

      return result;
    },
    endTime, dt
  );
}


std::vector<CartesianPose> BezierTrajectory::poses() const
{
  return smoothCurveTrajectory_->poses();
}

Eigen::Matrix6dynd BezierTrajectory::poseVelocities() const
{
  return smoothCurveTrajectory_->poseVelocities();
}

/** \brief interface to get sample period dt [s] */
double BezierTrajectory::dt() const
{
  return smoothCurveTrajectory_->dt();
}

/** \brief interface to get calculated end time [s] */
double BezierTrajectory::endTime() const
{
  return smoothCurveTrajectory_->endTime();
}

double BezierTrajectory::basis(int i, int n, double x) const
{
  assert(!knotVector_.empty());

  // left end of domain
  if (n == p_ && i == 0 && x < knotVector_[0])
  {
    return 1;
  }
    
  // right end
  if (n == p_ && i == (int)poses_.size()-1 && x >= knotVector_.back())
  {
    return 1;
  }
  
  if (n == 0)
  {
    if (knotVector_[i] <= x && x < knotVector_[i+1])
    {
      return 1;
    }
    return 0;
  }
  else
  {
    // compute first factor
    double nominator1 = (x - knotVector_[i]);
    double denominator1 = (knotVector_[i+n] - knotVector_[i]);
    
    double factor1 = 0;     // here, 0/0 =: 0
    if (fabs(denominator1) > 1e-12)
    {
      factor1 = nominator1 / denominator1;
    }
    
    // compute second factor
    double nominator2 = (knotVector_[i+n+1] - x);
    double denominator2 = (knotVector_[i+n+1] - knotVector_[i+1]);
    
    double factor2 = 0;
    if (abs(denominator2) > 1e-12)
    {
      factor2 = nominator2 / denominator2;
    }
      
    return factor1 * basis(i,n-1,x) + factor2 * basis(i+1,n-1,x);
  }
}

void BezierTrajectory::generateKnotVector()
{
  //p_ = 3
  //targetMultiplicity_ = 2

  int nPoints = poses_.size();
  int nBasisFunctions = nPoints;
  int k = nBasisFunctions + p_ + 1;   // length of knot vector
  int maxKnot = (int)(ceil((k - 2*(p_+1)) / float(targetMultiplicity_) + 1));
  int nBasisFunctionsWithMaxKnot = (maxKnot-1) * targetMultiplicity_ + p_ + 1;

  int nMissingFunctions = nBasisFunctionsWithMaxKnot - nPoints;
  int lastMultiplicity = targetMultiplicity_ - nMissingFunctions;

  // create knot vector

  knotVector_.resize(k);

  // start and end is p+1 times 0 and 1, respectively
  for (int i = 0; i < p_+1; i++)
  {
    knotVector_[i] = 0;
    knotVector_[k-p_-1 + i] = 1;
  }

  // Knots with multiplicity of targetMultiplicity_
  int index = p_ + 1;
  for (int i = 1; i < maxKnot-1; i++)
  {
    for (int j = 0; j < targetMultiplicity_; j++, index++)
    {
      knotVector_[index] = i / double(maxKnot);
    }
  }

  // last Knot with multiplicity of lastMultiplicity
  for (int j = 0; j < lastMultiplicity; j++, index++)
  {
    knotVector_[index] = (maxKnot-1) / double(maxKnot);
  }

  nBasisFunctions_ = knotVector_.size() - p_ - 1;

  std::cout << "p: " << p_ << ", multiplicity: " << targetMultiplicity_ << ", nBasisFunctions: " << nBasisFunctions << std::endl;
  std::cout << "Knotvector: ";
  for (int i = 0; i < (int)knotVector_.size(); i++)
  {
    if (i != 0)
      std::cout << ", ";
    std::cout << knotVector_[i];
  }
  std::cout << std::endl;
}