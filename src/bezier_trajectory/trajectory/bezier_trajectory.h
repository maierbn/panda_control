#pragma once

#include "utility/eigen_utility.h"
#include "trajectory/smooth_curve_trajectory.h"

/** \brief implements a trajectory following a bezier curve
 */
class BezierTrajectory : public SmoothCurveTrajectory
{
public:

  /** \brief 
	 */
  BezierTrajectory(CartesianPose initialPose, std::vector<CartesianPose> &poses, double p, double continuity, double endTime, double dt);

private:

  //SmoothCurveTrajectory smoothCurveTrajectory

	//! B-spline ansatz function B_i,n where i >= 0 is the number, n is the order, x is the coordinate
	double basis(int i, int n, double x);

  //! create the knot vector from p_, poses_ and targetMultiplicity_
	void generateKnotVector();

	std::vector<CartesianPose> &poses_;   //< the provided poses that should be interpolated
	double p_;   													//< polynomial degree of the ansatz functions
	double targetMultiplicity_;          //< multiplicity of the knots, degree of continuity = p - multiplicity
	std::vector<double> knotVector_;      //< knot vector for the spline curve
	double nBasisFunctions_ = 0;							//< number of poses = number of basis functions
};
