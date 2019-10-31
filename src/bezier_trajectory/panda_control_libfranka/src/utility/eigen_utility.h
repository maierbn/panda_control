#include <array>

#include <eigen3/Eigen/Eigen>

namespace Eigen {

/// 6d vector representation (used for [x,y,z,roll_x,pitch_y,yaw_z])
typedef Matrix<double, 6, 1> Vector6d;

/// 7d vector representation (used for [x,y,z,qx,qy,qz,qw])
typedef Matrix<double, 7, 1> Vector7d;

/// Matrix with 6 rows stored column-major, on column can be a pose velocity in [x,y,z,roll_x,pitch_y,yaw_z]
typedef Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> Matrix6dynd;

/// Matrix with 7 rows stored column-major, on column can be a pose velocity in [x,y,z,qx,qy,qz,qw]
typedef Matrix<double, 7, Eigen::Dynamic, Eigen::ColMajor> Matrix7dynd;

} // namespace Eigen
