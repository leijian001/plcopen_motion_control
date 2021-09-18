# RTmotion
PLCopen motion library

## Documentation

The documentation can be compiled to `doc` subfolder through the commands:
```bash
sudo apt install graphviz

doxygen ./Doxyfile
```

Open `doc/html/index.html` to check the documentation.

- [S-curve trajectory planning](./src/algorithm/README.md)

## Continous Integration

Travis CI: ![](https://api.travis-ci.com/RoboticsYY/plcopen_motion_control.svg?token=xy7TUUdhEnnR5zpRg3g5&branch=master)

Gitlab CI: [![pipeline status](https://gitlab.devtools.intel.com/iotg-china-ist/plcopen_motion_control/badges/master/pipeline.svg)](https://gitlab.devtools.intel.com/iotg-china-ist/plcopen_motion_control/-/commits/master)

## Install dependency

```bash
sudo apt install libeigen3-dev

# Install ruckig
git clone https://github.com/pantor/ruckig.git

cd <path_to_ruckig>
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make && sudo make install
```

## Build

```bash
git clone https://gitlab.devtools.intel.com/iotg-china-ist/plcopen_motion_control.git

cd <path_to_plcopen_motion_control>

mkdir build && cd build

# If build on Ubuntu Linux, run the next command
cmake ..

# If build on PREEMPT Linux, run the next command
cmake ..

# If build on Xenomai, run the next command
cmake .. -DXENOMAI_DIR=/usr/xenomai/bin/

make && sudo make install

# Try **sudo ldconfig** after installation if meet any problem related to library file missing.
```
> Note: Install **googletest** then add `-DTEST=ON -DPLOT=ON -DSYSTEM_ROOT=/usr` at the end of cmake command if need to do Unit Test and plot the result. Check Unit Test below for details.

## Run Demo

```bash
single_axis_move_relative
```

## Run Evaluation

Running the evaluation program using the following commands:

```bash
axis_move_cyclic
```

This evaluation program enables a MC_MoveRelative function block running in 1ms real-time cycle. The function block will be re-triggered everytime it finished its task. It can be stopped by `Ctrl+C`.

## Unit Test

- Googletest is used as the test framework. Therefore, if test running is desired, follow the commands below to install `gtest` at first.
  ```bash
  sudo apt install googletest
  ```
  or

  ```bash
  export CXXFLAGS="-std=c++11"
  cd ~ && git clone https://github.com/google/googletest.git
  cd googletest && mkdir build
  cd build
  cmake .. -DBUILD_SHARED_LIBS=ON
  sudo make install
  ```

  In order to build the tests, add the argument `-DTEST=On` to `cmake` commands.

- To enable the plotting in the test, install the dependencies below.

  ```bash
  sudo apt-get install python3-matplotlib python3-numpy python3-dev
  ```

  And add CMake argument `-DPLOT=On`.

- On-line S-Curve Algorithm Test:
  ```bash
  <build folder>/test/online_scurve_test
  ```

- Trjactory Planner Test:
  ```bash
  <build folder>/test/planner_test
  ```

- Function Block Test:
  ```bash
  <build folder>/test/function_block_test
  ```

## Link to the library

Please find all headers files under `/usr/local/include/RTmotion/`. The library file `libRTmotion.so` locates at `/use/local/lib`.

