# Sample implementation to test the `libfranka` C++ interface for simple linear movements

[![pipeline status](https://git.isw.uni-stuttgart.de/projekte/forschung/2017_DFG_IRTG_SoftTissueRobotics/panda_control_libfranka/badges/master/pipeline.svg)](https://git.isw.uni-stuttgart.de/projekte/forschung/2017_DFG_IRTG_SoftTissueRobotics/panda_control_libfranka/commits/master)

Depends on [`Eigen3`](http://eigen.tuxfamily.org/index.php?title=Main_Page) and [`libfranka`](https://frankaemika.github.io/docs/libfranka.html#)

## Install and run:

Set up the dependencies: We need `libfranka` (unfortunately not directly available through sources) and `libeigen3-dev`

```sh
mkdir -p ~/dev && cd ~/dev
libfranka_version="0.6.0"

# install Eigen3 and libfranka 
sudo apt update && sudo apt install --yes build-essential cmake git libpoco-dev libeigen3-dev

git clone https://github.com/frankaemika/libfranka.git
cd libfranka

git checkout ${libfranka_version}
git submodule update

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ..
make -j4
sudo make install
```

Finally, install this project:
```sh
# install rapidjson for serialization
sudo apt install rapidjson-dev
cd ~/dev
git clone git@git.isw.uni-stuttgart.de:projekte/forschung/2017_DFG_IRTG_SoftTissueRobotics/panda_control_libfranka.git
mkdir panda_control_libfranka/build && cd panda_control_libfranka/build

# generate makefiles from cmake
cmake ..
cmake -DCMAKE_BULD_TYPE=Debug .. # if e.g. unittests should be debugged. Do not use this when operating hte robot.

#compile
make

# to run the unittests
make tests

# execute the binary (robot needs to be connected)
./src/test_franka_ctrl

```