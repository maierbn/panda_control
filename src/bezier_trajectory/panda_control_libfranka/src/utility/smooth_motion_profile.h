#pragma once

#include "utility/eigen_utility.h"

#include <algorithm>
#include <eigen3/Eigen/Eigen>

/** \brief 3 times differentiable polynomial reference trajectory z(t) \in [0,1]
 *
 * z(t) = a_4*(t/T)^4 + a_5*(t/T)^5 +a_6*(t/T)^6 + a_7*(t/T)^7
 *
 * with a_4 = 35, a_5 = -84, a_6 = 70, a_7 = -20
 *
 * */
class SmoothMotionProfile
{
public:
  /** \brief Evaluates the motion profile for the whole trajectory with normalized length coordinate
   * s \in [0,1]
   *
   * \arg[in] vMax_allowed maximum allowed velocity [m/s], the trajectory is adapted to this limit
   * \arg[in] aMax_allowed maximum allowed acceleration [m/s^2], the trajectory is adapted to this
   * limit \arg[in] L the real length of the calling curve p(s) [m] \arg[in] dt time step width [s]
   *
   * \returns position values
   *
   * */
  static Eigen::VectorXd s_t(double vMax_allowed, double aMax_allowed, double L, double dt);

  /** \brief Evaluates the derivative of the motion profile for the whole trajectory w.r.t the
   * normalized length coordinate s \in [0,1]
   *
   * \arg[in] vMax_allowed maximum allowed velocity [m/s], the trajectory is adapted to this limit
   * \arg[in] aMax_allowed maximum allowed acceleration [m/s^2], the trajectory is adapted to this
   * limit \arg[in] L the real length of the calling curve [m] \arg[in] dt time step width [s]
   *
   * \returns velocity values
   *
   * */
  static Eigen::VectorXd ds_dt(double vMax_allowed, double aMax_allowed, double L, double dt);

  /** \brief Evaluate smooth motion profile for end time 1.0
   * 
   * \arg[in] t evaluation time 
   */
  static double evaluate(double t);

  /** \brief returns the end time of the motion profile depending on curve length L and allowed
   * dynamics limits
   * */
  static double endTime(double vMax_allowed, double aMax_allowed, double L, double dt);

private:
  /// the coefficients of the polynomial (a_4 - a_7)
  static const Eigen::Vector4d poly_coeffs;

  /// the factors obtained by 1st derivative of the polynomial exponents
  static const Eigen::Vector4d a_d_mult;

  // static const Eigen::Vector4d a_dd_mult(3,4,5,6);

  static constexpr double p_v_max = 2.1875;
  static constexpr double p_a_max = 7.513188404399293;

};
