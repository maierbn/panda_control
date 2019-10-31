#include "bezier_trajectory.h"

#include <iostream>

BezierTrajectory::BezierTrajectory(CartesianPose initialPose, std::vector<CartesianPose> &poses, double p, double continuity, double endTime, double dt) :
   poses_(poses), p_(p), targetMultiplicity_(p - continuity),
   SmoothCurveTrajectory(initialPose,
    [initialPose, poses, endTime, this](double time) -> CartesianPose
    {
      double t = time/endTime;

      CartesianPose result;
      for (int i = 0; i < this->nBasisFunctions_; i++)
      {
        result += poses_[i] * basis(i,p_,t);
      }

      return result;
    },
    endTime, dt
  )
{
}

double BezierTrajectory::basis(int i, int n, double x)
{
  if (knotVector_.empty())
  {
    generateKnotVector();
  }

  // left end of domain
  if (n == p_ && i == 0 && x < knotVector_[0])
  {
    return 1;
  }
    
  // right end
  if (n == p_ && i == poses_.size()-1 && x >= knotVector_.back())
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
    knotVector_[k-p_-1 + i] = 0;
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
  for (int i = 0; i < knotVector_.size(); i++)
  {
    if (i != 0)
      std::cout << ", ";
    std::cout << knotVector_[i];
  }
  std::cout << std::endl;
}