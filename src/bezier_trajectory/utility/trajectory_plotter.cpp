#include "trajectory_plotter.h"

#include <iostream>

#include "utility/eigen_utility.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <fstream>

TrajectoryPlotter::TrajectoryPlotter(CartesianPose startPose, std::shared_ptr<Trajectory> trajectory, double samplingTimestepWidth)
{
  std::cout << "get poseVelocities" << std::endl;
  // compute poses from poseVelocities to immitate trajectory_iterator_cartesian_velocity
  Eigen::Matrix6dynd poseVelocities = trajectory->poseVelocities();

  int nEntries = poseVelocities.cols();
  std::cout << "nEntries: " << nEntries << std::endl;

  const double dt = samplingTimestepWidth;

  // start python script text
  std::stringstream pythonScript;

  pythonScript << "#!/usr/bin/python\n\n" 
    << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n"
    << "from mpl_toolkits.mplot3d import Axes3D\n\n"
    << "# [x,y,z,phi,theta]\n"
    << "poses = [\n";

  // integrate poses from velocities
  CartesianPose currentPose = startPose;

  std::cout << "start pose: " << currentPose << std::endl;

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

  // define rest of python script to plot
  pythonScript << "]\n" << R"(
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
stride = 100
n_poses = len(poses)
for i in range(0,n_poses,stride):

  # get current pose
  pose = poses[i]
  [x,y,z,yaw,pitch] = pose

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

# plot path 
ax.plot(path_x_list, path_y_list, path_z_list, lw="2", color="red")
  
# add text "start" and "end" at ends of path
ax.text(path_x_list[0], path_y_list[0], path_z_list[0], "start", color='red')
ax.text(path_x_list[-1], path_y_list[-1], path_z_list[-1], "end", color='red')
  
# make axes equal and add labels
set_axes_equal(ax)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# show plot window
plt.show()

  )";

  std::string filename = "plot_poses.py";

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
  int returnValue = system("chmod +x ./plot_poses.py; ./plot_poses.py");

  if (returnValue != 0)
  {
    std::cout << "Could not execute plot script." << std::endl;
  }
  std::cout << "Continue program." << std::endl;
}

void TrajectoryPlotter::plot()
{
  
}
