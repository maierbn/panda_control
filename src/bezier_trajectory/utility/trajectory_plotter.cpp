#include "trajectory_plotter.h"

#include <iostream>
#include <iomanip>

#include "utility/eigen_utility.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <fstream>

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth)
{
  std::cout << "get poseVelocities" << std::endl;
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  std::stringstream pythonScript;
  pythonScript << R"(#!/usr/bin/python
# -*- coding: UTF-8 -*-
)";

  std::vector<CartesianPose> poses;
  std::vector<double> knots;
  pythonScript << plotPoseVelocities(poseVelocities, samplingTimestepWidth);
  pythonScript << plotTrajectory(startPose, poseVelocities, samplingTimestepWidth, poses, knots);

  pythonScript << R"(

# show plot window
plt.show()
)";

  std::string filename = "plot.py";

  // write resulting python script to file
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cout << "Could not write to file \"" << filename << "\"." << std::endl;
    return;
  }

  file << pythonScript.str();
  file.close();

  std::cout << "wrote file \"" << filename << "\"" << std::endl;

  // execute plot script
  std::cout << "Plotting trajectory, close plot window to continue." << std::endl;
  int returnValue = system("chmod +x ./plot.py; ./plot.py");

  if (returnValue != 0)
  {
    std::cout << "Could not execute plot script." << std::endl;
  }
  std::cout << "Continue program." << std::endl;

}

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory,
                    const std::vector<CartesianPose> &poses, double samplingTimestepWidth)
{
  std::cout << "get poseVelocities" << std::endl;
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  std::stringstream pythonScript;
  pythonScript << R"(#!/usr/bin/python
# -*- coding: UTF-8 -*-
)";

  std::vector<double> knots;

  pythonScript << plotPoseVelocities(poseVelocities, samplingTimestepWidth);
  pythonScript << plotTrajectory(startPose, poseVelocities, samplingTimestepWidth, poses, knots);

  pythonScript << R"(

# show plot window
plt.show()
)";

  std::string filename = "plot.py";

  // write resulting python script to file
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cout << "Could not write to file \"" << filename << "\"." << std::endl;
    return;
  }

  file << pythonScript.str();
  file.close();

  std::cout << "wrote file \"" << filename << "\"" << std::endl;

  // execute plot script
  std::cout << "Plotting trajectory, close plot window to continue." << std::endl;
  int returnValue = system("chmod +x ./plot.py; ./plot.py");

  if (returnValue != 0)
  {
    std::cout << "Could not execute plot script." << std::endl;
  }
  std::cout << "Continue program." << std::endl;

}

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory,
                    const std::vector<CartesianPose> &poses, const std::vector<double> &knots, double samplingTimestepWidth)
{
  std::cout << "get poseVelocities" << std::endl;
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  std::stringstream pythonScript;
  pythonScript << R"(#!/usr/bin/python
# -*- coding: UTF-8 -*-
)";

  pythonScript << plotPoseVelocities(poseVelocities, samplingTimestepWidth);
  pythonScript << plotTrajectory(startPose, poseVelocities, samplingTimestepWidth, poses, knots);

  pythonScript << R"(

# show plot window
plt.show()
)";

  std::string filename = "plot.py";

  // write resulting python script to file
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cout << "Could not write to file \"" << filename << "\"." << std::endl;
    return;
  }

  file << pythonScript.str();
  file.close();

  std::cout << "wrote file \"" << filename << "\"" << std::endl;

  // execute plot script
  std::cout << "Plotting trajectory, close plot window to continue." << std::endl;
  int returnValue = system("chmod +x ./plot.py; ./plot.py");

  if (returnValue != 0)
  {
    std::cout << "Could not execute plot script." << std::endl;
  }
  std::cout << "Continue program." << std::endl;

}

std::string TrajectoryPlotter::plotTrajectory(CartesianPose startPose, const Eigen::Matrix6dynd &poseVelocities,
                                              double dt, const std::vector<CartesianPose> &poses, const std::vector<double> &knots)
{
  // start python script text
  std::stringstream pythonScript;

  pythonScript << "\n\n"
    << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n"
    << "from mpl_toolkits.mplot3d import Axes3D\n\n"
    << "# [x,y,z,phi,theta]\n"
    << "path_poses = [\n";

  // integrate poses from velocities
  CartesianPose currentPose = startPose;

  std::cout << "start pose: " << currentPose << std::endl;

  int nEntries = poseVelocities.cols();
  std::cout << "nEntries: " << nEntries << std::endl;

  // add poses from path
  for (int i = 0; i < nEntries; i++)
  {
    double currentTime = i * dt;
    Eigen::Vector6d poseVelocity = poseVelocities.col(i);

    // transform rotational velocity to quaternions
    double yaw = poseVelocity[5] * dt;      // about z-axis
    double pitch = poseVelocity[4] * dt;    // about y-axis
    double roll = poseVelocity[3] * dt;     // about x-axis

    Eigen::Quaterniond rotation;
    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // get linear velocity
    Eigen::Vector3d linearVelocity = poseVelocity.head<3>();

    // add contribution from velocity to current pose
    currentPose.position += linearVelocity * dt;
    currentPose.orientation = rotation * currentPose.orientation;

    Eigen::AngleAxisd plotRotation2(-M_PI/2., Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd plotRotation(M_PI/2., Eigen::Vector3d::UnitY());
    Eigen::Quaterniond orientationToPlot = plotRotation2 * plotRotation * currentPose.orientation;

    // output current pose to python script
    Eigen::Vector3d eulerAngles = orientationToPlot.toRotationMatrix().eulerAngles(2,1,0);

    pythonScript << "\t[" << currentPose.position[0] << "," << currentPose.position[1] << "," << currentPose.position[2] << ","
      << eulerAngles[0] << "," << M_PI/2. - eulerAngles[1] << "],    # t: " << currentTime
      << " ypr: (" << orientationToPlot.toRotationMatrix().eulerAngles(2,1,0).transpose() / M_PI * 180.
      << "), phi: " << eulerAngles[0] / M_PI * 180. << ", theta: " << (M_PI/2. - eulerAngles[1]) / M_PI * 180 << std::endl;
  }

  pythonScript << "]\n\n"
    << "extra_poses = [\n";

  // add extra poses
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    CartesianPose currentPose = startPose -poses[0] + poses[i];

    Eigen::AngleAxisd plotRotation2(-M_PI/2., Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd plotRotation(M_PI/2., Eigen::Vector3d::UnitY());
    Eigen::Quaterniond orientationToPlot = plotRotation2 * plotRotation * currentPose.orientation;

    // output current pose to python script
    Eigen::Vector3d eulerAngles = orientationToPlot.toRotationMatrix().eulerAngles(2,1,0);

    pythonScript << "\t[" << currentPose.position[0] << "," << currentPose.position[1] << "," << currentPose.position[2] << ","
      << eulerAngles[0] << "," << M_PI/2. - eulerAngles[1] << "],    # i: " << i
      << " ypr: (" << orientationToPlot.toRotationMatrix().eulerAngles(2,1,0).transpose() / M_PI * 180.
      << "), phi: " << eulerAngles[0] / M_PI * 180. << ", theta: " << (M_PI/2. - eulerAngles[1]) / M_PI * 180 << std::endl;
  }


  const int stride = std::max(1,(int)(nEntries/100));

  // define rest of python script to plot
  pythonScript << "]\n\n" << R"(
# source: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)

# setup the 3d figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('equal')

# lists to hold path
path_x_list = []
path_y_list = []
path_z_list = []

# loop over poses with given stride
stride = )" << stride << R"(
n_path_poses = len(path_poses)
for i in range(0,n_path_poses,stride):

  # get current pose
  path_pose = path_poses[i]
  [x,y,z,yaw,pitch] = path_pose

  # add pose to list of coordinates for final path
  path_x_list.append(x)
  path_y_list.append(y)
  path_z_list.append(z)

  # define small line that visualizes orientation
  size = 0.02
  x_list = [x, x + size*np.sin(np.pi/2.-pitch)*np.cos(yaw)]
  y_list = [y, y + size*np.sin(np.pi/2.-pitch)*np.sin(yaw)]
  z_list = [z, z + size*np.cos(np.pi/2.-pitch)]

  # plot orientation line
  ax.plot(x_list, y_list, z_list)

# plot extra poses
for i in range(len(extra_poses)):

  # get pose
  [x,y,z,yaw,pitch] = extra_poses[i]

  # define small line that visualizes orientation
  size = 0.04
  x_list = [x, x + size*np.sin(np.pi/2.-pitch)*np.cos(yaw)]
  y_list = [y, y + size*np.sin(np.pi/2.-pitch)*np.sin(yaw)]
  z_list = [z, z + size*np.cos(np.pi/2.-pitch)]

  # plot orientation line of extra poses
  ax.plot(x_list, y_list, z_list, 'k-', lw=1, label="poses")

# plot path
ax.plot(path_x_list, path_y_list, path_z_list, lw="2", color="red", label="interpolated path")

# add text "start" and "end" at ends of path
ax.text(path_x_list[0], path_y_list[0], path_z_list[0], "start", color='red')
ax.text(path_x_list[-1], path_y_list[-1], path_z_list[-1], "end", color='red')

# make axes equal and add labels
set_axes_equal(ax)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("n poses in path: {}, stride for this plot: {}".format(n_path_poses, stride))
ax.legend(loc="lower left")
plt.savefig('trajectory.pdf')

)";

if (!knots.empty())
{
  pythonScript << R"(

dt = )" << dt << R"(

knot_list = [)";

for (int i = 0; i < knots.size(); i++)
{
  pythonScript << knots[i] << ",";
}
pythonScript << "]\n";

pythonScript << R"(

# create 2x3 subplots
fig, axes = plt.subplots(3,2)

if False:
  xlist = [i*dt for i in range(len(path_poses))]
  axes[0,0].plot(xlist, [x for [x,y,z,phi,theta] in path_poses], label="x")
  axes[0,0].plot(knot_list, [x for [x,y,z,phi,theta] in extra_poses], "ro")
  axes[0,0].set_title("x")

  axes[0,1].plot(xlist, [y for [x,y,z,phi,theta] in path_poses], label="y")
  axes[0,1].set_title("y")

  axes[0,2].plot(xlist, [z for [x,y,z,phi,theta] in path_poses], label="z")
  axes[0,2].set_title("z")

  axes[1,0].plot(xlist, [phi for [x,y,z,phi,theta] in path_poses], label="phi")
  axes[1,0].set_title("phi")

  axes[1,1].plot(xlist, [theta for [x,y,z,phi,theta] in path_poses], label="theta")
  axes[1,1].set_title("theta")




plt.savefig('trajectory_components.pdf')

  )";
}
  return pythonScript.str();
}

std::string TrajectoryPlotter::plotPoseVelocities(const Eigen::Matrix6dynd &poseVelocities, double dt)
{
  const int nPoseVelocities = poseVelocities.cols();

  Eigen::Vector6d previousVelocity;
  Eigen::Vector6d previousAcceleration;

  std::vector<Eigen::Vector6d> velocities;
  std::vector<Eigen::Vector6d> accelerations;
  std::vector<Eigen::Vector6d> jerks;
  std::vector<double> timePoints;

  velocities.reserve(nPoseVelocities);
  accelerations.reserve(nPoseVelocities);
  jerks.reserve(nPoseVelocities);
  timePoints.reserve(nPoseVelocities);

  std::cout << "checkPoseVelocities" << std::endl;
  for (int timeStepNo = 0; timeStepNo < nPoseVelocities; timeStepNo++)
  {
    // (v_x, v_y, v_z, omega_x, omega_x, omega_z), v in [m/s], omega in [rad/s]
    const Eigen::Vector6d velocity = poseVelocities.col(timeStepNo);

    // compute acceleration
    const Eigen::Vector6d acceleration = (velocity - previousVelocity) / dt;

    const Eigen::Vector6d jerk = (acceleration - previousAcceleration) / dt;

    // jerk is computed after timeStepNo = 1
    if (timeStepNo > 1)
    {
      const double currentTime = timeStepNo * dt;

      jerks.push_back(jerk);
      accelerations.push_back(acceleration);
      velocities.push_back(velocity);
      timePoints.push_back(currentTime);

      bool jerkIsTooHigh = false;
      for (int i = 0; i < 6; i++)
      {
        if (fabs(jerk[i]) > 1e-3)
        {
          jerkIsTooHigh = true;
        }
      }
#if 0

      // output to console
      std::cout << std::setw(4) << timeStepNo << std::setw(7) << std::setprecision(3) << std::setfill(' ') << std::scientific << std::showpos
        << ": v [" << velocity[0] << "," << velocity[1]  << "," << velocity[2]
        << "," << velocity[3] << "," << velocity[4] << "," << velocity[5] << "]"
        << ", a [" << acceleration[0] << "," << acceleration[1]  << "," << acceleration[2]
        << "," << acceleration[3] << "," << acceleration[4] << "," << acceleration[5] << "]"
        << ", j [" << jerk[0] << "," << jerk[1]  << "," << jerk[2]
        << "," << jerk[3] << "," << jerk[4] << "," << jerk[5] << "]" << std::endl;
#endif
    }

    previousAcceleration = acceleration;
    previousVelocity = velocity;
  }

  std::stringstream pythonScript;
  pythonScript << R"(#!/usr/bin/python
# -*- coding: UTF-8 -*-

import numpy as np
import matplotlib.pyplot as plt

time_list = [)";

  for (unsigned int i = 0; i < timePoints.size(); i++)
  {
    pythonScript << timePoints[i] << ",";
  }
  pythonScript << "]\n";

  // write velocities
  pythonScript << "velocity_lists = [";
  for (unsigned int componentNo = 0; componentNo < 6; componentNo++)
  {
    pythonScript << "  [";
    for (unsigned int i = 0; i < velocities.size(); i++)
    {
      pythonScript << velocities[i][componentNo] << ",";
    }
    pythonScript << "]" << (componentNo != 5? "," : "") << "\n";
  }
  pythonScript << "]\n";

  // write accelerations
  pythonScript << "acceleration_lists = [";
  for (unsigned int componentNo = 0; componentNo < 6; componentNo++)
  {
    pythonScript << "  [";
    for (unsigned int i = 0; i < accelerations.size(); i++)
    {
      pythonScript << accelerations[i][componentNo] << ",";
    }
    pythonScript << "]" << (componentNo != 5? "," : "") << "\n";
  }
  pythonScript << "]\n";

  // write jerks
  pythonScript << "jerk_lists = [";
  for (unsigned int componentNo = 0; componentNo < 6; componentNo++)
  {
    pythonScript << "  [";
    for (unsigned int i = 0; i < jerks.size(); i++)
    {
      pythonScript << jerks[i][componentNo] << ",";
    }
    pythonScript << "]" << (componentNo != 5? "," : "") << "\n";
  }
  pythonScript << "]\n";



  pythonScript << R"(
# create 3x2 subplots
fig, axes = plt.subplots(3,2)

# plot velocities
labels = ["vx","vy","vz",u"ωx",u"ωy",u"ωz"]
for i in range(3):
  axes[0,0].plot(time_list, velocity_lists[i], label=labels[i])
axes[0,0].set_title("linear velocity")
axes[0,0].legend()
axes[0,0].grid(which='major')

for i in range(3,6):
  axes[0,1].plot(time_list, velocity_lists[i], label=labels[i])
axes[0,1].set_title("angular velocity")
axes[0,1].legend()
axes[0,1].grid(which='major')

# plot accelerations
labels = ["ax","ay","az",u"αx",u"αy",u"αz"]
for i in range(3):
  axes[1,0].plot(time_list, acceleration_lists[i], label=labels[i])
axes[1,0].set_title("linear acceleration")
axes[1,0].legend()
axes[1,0].grid(which='major')

for i in range(3,6):
  axes[1,1].plot(time_list, acceleration_lists[i], label=labels[i])
axes[1,1].set_title("angular acceleration")
axes[1,1].legend()
axes[1,1].grid(which='major')

# plot jerks
labels = ["jx","jy","jz","jx","jy","jz"]
for i in range(3):
  axes[2,0].plot(time_list, jerk_lists[i], label=labels[i])
axes[2,0].set_title("linear jerk")
axes[2,0].legend()
axes[2,0].grid(which='major')

for i in range(3,6):
  axes[2,1].plot(time_list, jerk_lists[i], label=labels[i])
axes[2,1].set_title("angular jerk")
axes[2,1].legend()
axes[2,1].grid(which='major')

plt.savefig('values.pdf')

)";

  return pythonScript.str();
}
