#pragma once

#include "utility/eigen_utility.h"
#include "trajectory/smooth_curve_trajectory.h"

/** \brief implements a trajectory following a bezier curve
 */
class BezierTrajectory : public Trajectory
{
public:

  /** \brief 
	 */
  BezierTrajectory(CartesianPose initialPose, std::vector<CartesianPose> &poses, double p, double continuity, double endTime, double dt);

  /** \brief interface to get the pose values column-wise for the whole trajectory, sampled with dt, orientation as quaternions
   */
  std::vector<CartesianPose> poses() const;

  /** \brief interface to get the pose velocity values column-wise for the whole trajectory, sampled
   * with dt, orientations as Euler angles */
  Eigen::Matrix6dynd poseVelocities();

  /** \brief interface to get sample period dt [s] */
  double dt() const;

  /** \brief interface to get calculated end time [s] */
  double endTime() const;

  //! return all distinct knot values
  void getKnots(std::vector<double> &knots) const;

private:

	std::shared_ptr<SmoothCurveTrajectory> smoothCurveTrajectory_;  //< the smooth curve trajectory

	//! B-spline ansatz function B_i,n where i >= 0 is the number, n is the order, x is the coordinate
	double basis(int i, int n, double x) const;

  //! create the knot vector from p_, poses_ and targetMultiplicity_
	void generateKnotVector();

	CartesianPose initialPose_;           //< the pose from which the trajectory starts
	std::vector<CartesianPose> &poses_;   //< the provided poses that should be interpolated
	std::vector<double> knotVector_;      //< knot vector for the spline curve
	double p_;   													//< polynomial degree of the ansatz functions
	double nBasisFunctions_;							//< number of poses = number of basis functions
	double targetMultiplicity_;          //< multiplicity of the knots, degree of continuity = p - multiplicity
};
