#include "utility/smooth_motion_profile.h"

// the coefficients of the polynomial (a_4 - a_7)
const Eigen::Vector4d SmoothMotionProfile::poly_coeffs(35., -84., 70., -20.);

// the factors obtained by 1st derivative of the polynomial exponents
const Eigen::Vector4d SmoothMotionProfile::a_d_mult(4., 5., 6., 7.);

double SmoothMotionProfile::endTime(double vMax_allowed, double aMax_allowed, double L,
                                        double dt) {
  double t_E = std::max(p_v_max * L / vMax_allowed, sqrt(p_a_max * L / aMax_allowed)); // end time

  return ceil(t_E / dt) * dt; // round t_E to whole dt steps number
}

double SmoothMotionProfile::evaluate(double t)
{
  std::size_t N_poly = poly_coeffs.size();

  double result = 0.0;
  for (std::size_t i = 0; i < N_poly; i++) {
    double expCoeff = N_poly + i;
    result += poly_coeffs[i] * std::pow(t, expCoeff);
  }

  return result;
}

Eigen::VectorXd SmoothMotionProfile::s_t(double vMax_allowed, double aMax_allowed, double L, double dt)
{
  if (vMax_allowed < 0 || aMax_allowed < 0 || L < 0 || dt < 0) {
    throw std::invalid_argument(
        "SmoothMotionProfile::s_t. Parameters are required to be positive.");
  }

  double t_E = endTime(vMax_allowed, aMax_allowed, L, dt);
  std::size_t N_poly = poly_coeffs.size();

  Eigen::VectorXd t =
      Eigen::VectorXd::LinSpaced(t_E / dt + 1, 0, t_E) / t_E; // generate normalized time vector
  Eigen::MatrixXd _T(N_poly, t.size());

  for (std::size_t i = 0; i < N_poly; i++)
  {
    double expCoeff = N_poly + i;
    _T.row(i) = t.array().pow(expCoeff);
  }

  Eigen::VectorXd s = poly_coeffs.transpose() * _T;
  return s;
}

Eigen::VectorXd SmoothMotionProfile::ds_dt(double vMax_allowed, double aMax_allowed, double L, double dt)
{
  if (vMax_allowed < 0 || aMax_allowed < 0 || L < 0 || dt < 0) {
    throw std::invalid_argument(
        "SmoothMotionProfile::s_t. Parameters are required to be positive.");
  }

  double t_E = endTime(vMax_allowed, aMax_allowed, L, dt);
  std::size_t N_poly = poly_coeffs.size();

  Eigen::VectorXd t =
      Eigen::VectorXd::LinSpaced(t_E / dt + 1, 0, t_E) / t_E; // generate normalized time vector
  Eigen::MatrixXd _T(N_poly, t.size());

  for (std::size_t i = 0; i < N_poly; i++) {
    double expCoeff = N_poly + i - 1;
    _T.row(i) = t.array().pow(expCoeff);
  }

  Eigen::VectorXd deriv_poly_coeffs = poly_coeffs.cwiseProduct(a_d_mult) / t_E;

  Eigen::VectorXd ds = deriv_poly_coeffs.transpose() * _T;
  return ds;
}
