# cHyRRT
A C++ hybrid rapidly-exploring random tree motion planner, compatible with OMPL and ROS 2 Humble.

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-22.04-orange?logo=Ubuntu&Ubuntu-22.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-humble-blue?logo=ROS&ROS=humble" alt="ROS" />
</p>

# Hybrid Rapidly-Exploring Random Trees (HyRRT)

**Motion Planning** is a computational problem that involves finding a sequence of valid configurations to move the robot from the source to the destination. 

This repository provides the implementation of **HyRRT** in C++, compatible with OMPL and ROS 2 Humble. Vertices are implemented as the datatype [ompl::base::State *](https://ompl.kavrakilab.org/classompl_1_1base_1_1State.html) and edges as vectors of the same datatype. The theory analysis of HyRRT can be found at [N. Wang and R. G. Sanfelice](https://ieeexplore.ieee.org/document/9992444). Furthermore, we already provide a [MATLAB](https://github.com/HybridSystemsLab/hybridRRT) version, without compatibility with OMPL and ROS.

**Your stars, forks and PRs are welcome!**

## Contents
- [Quick Start](#0)
- [Document](#1)
- [Acknowledgments](#2)
- [License](#3)
- [Maintenance](#4)

## <span id="0">0. Quick Start

*Tested on ubuntu 22.04.3 LTS with ROS Humble.*

1. Install [OMPL](https://ompl.kavrakilab.org/installation.html).

2. Install git.
    ```bash
    sudo apt install git
    ```

3. Install [ROS 2](https://docs.ros.org/en/humble/Installation.html) (Humble recommended, but later versions like Iron may also work.)

4. Clone the reposity.
    ```bash
    git clone https://github.com/xu21beve/cHyRRT.git
    ```


5. Setup CMake build Directory
  **Note: 
   Open your terminal and navigate to the root folder of this repository. Then, create a new directory called 'build' and set it as your build directory using CMake. You can do this using the following commands:
   ```
   cd <path-to-your-root-directory>
   mkdir build
   cd build
   cmake ..
   ```

2. Build the Project

   Once the CMake files have been generated in the `build` directory, you can compile all the source files into executables using the `make` command:
   ```
   make
   ```

3. Run the Executables

   Now, you can run the generated executable files located in the `build/examples` folder. You can do this using the following commands:

   - For the Bouncing Ball example, navigate to the project root directory and execute the `bouncing_ball` executable as follows:
     ```
     cd ..
     ./build/examples/bouncing_ball
     ```

   - For the multicopter example, execute the `multicopter` executable as follows:
     ```
     ./build/examples/multicopter
     ```

***Optional:***

7. Use ROS RViz2 to visualize the path.

  - Ensure that you have ROS Humble and colcon properly installed.
  - Navigate to the examples folder.
    ```
     cd examples/visualize
    ```
  - If you would like to visualize multicopter example with obstacles, uncomment lines 208-210
  - Paste the trajectory matrix output from running `bouncing_ball.cpp`, `multicopter.cpp`, or any other implementation of `HyRRT` into `points.txt`.
  - Run rosrun.bash, and follow the instructions within the terminal. Note that this visualization is limited to three dimentions. 
    ```
     ./rosrun.bash
    ```
    ***Include different version for different Operating Systems...***

## 1. <span id="1">Document

The overall modifiable file structure is shown below.
```
/root_directory
├── HyRRT.h
├── yourOtherFiles.cpp
└── src
    ├── HyRRT.cpp
    └── yourSrcFiles.cpp
└── examples
    ├── bouncing_ball.cpp
    ├── multicopter.cpp
    └── yourSrcFiles.cpp
```

For more information about the project's customizeable parameters, please refer to the following table.

| Required | Name | Description |
|:----:|:----:|:----:|
|Yes| maxInputValue_ | Vector of maximum input values (std::vector<double>)
|Yes| minInputValue_ | Vector of minimum input values (std::vector<double>)
|Yes| Tm_ | The maximum flow time for a given flow propagation step. (double)
|Yes| flowStepLength_ | The flow time for a given integration step, within a flow propagation step. (double)
|No| goalTolerance_ | The distance tolerance from the goal state for a state to be regarded as a valid final state. Default is .1 (double)
|Yes| jumpSet_ | Function that returns true if a state is in the jump set, and false if not. (std::function<bool(ompl::base::State *)>)
|Yes| flowSet_ | Function that returns true if a state is in the flow set, and false if not. (std::function<bool(ompl::base::State *)>)
|No| unsafeSet_ | Function that returns true if a state is in the unsafe set, and false if not. (std::function<bool(ompl::base::State )>)
|No| distanceFunc_ | Function that computes distance between states, default is Euclidean distance. (std::function<double(ompl::base::State *, ompl::base::State *)>)
|Yes| discreteSimulator_ | Jump map for propagating a state once. (std::function<ompl::base::State *(ompl::base::State *x_cur, double u, ompl::base::State *x_new)>)
|Yes| continuousSimulator_ | Flow map for propagating a state over the given flow time. (std::function<base::State *(std::vector<double> input, ompl::base::State *x_cur, double Tm_rand, ompl::base::State *x_new)>)
|No| collisionChecker_ | Function that returns true and modifies the state if collides into the obstacleSet. Default is point-by-point collision checking using the jump set. Refer to **HyRRT.h** for method signature. 

For more information about the project's available random sampling distributions for inputs, please refer to the following table. All descriptions and names are credited to [the Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/)
| Name | Description | Required Parameters |
|:----:|:----:|:----:|
|UNIFORM_INT| Uniform sampling of integers within between the given maximum and minimum integers. | Minimum integer value(s), maximum integers value(s)
|UNIFORM_REAL| Uniform sampling of real numbers within between the given maximum and minimum real numbers. | Minimum real number value(s), maximum real number value(s)
|GAUSSIAN_REAL| Generate a random real using a normal distribution with given mean and variance as real numbers. | Mean, variance
|HALF_NORMAL_REAL| Generate a random real using a half-normal distribution. The value is within specified bounds [ r_min, r_max], but with a bias towards r_max. The function is implemented using a Gaussian distribution with mean at r_max - r_min. The distribution is 'folded' around r_max axis towards r_min. The variance of the distribution is (r_max - r_min) / focus. The higher the focus, the more probable it is that generated numbers are close to r_max. | r_min, r_max, focus (default is 3.0)
|HALF_NORMAL_INT| Generate a random integer using a half-normal distribution. The value is within specified bounds ([r_min, r_max]), but with a bias towards r_max. The function is implemented on top of halfNormalReal() | r_min, r_max, focus (default is 3.0)


## <span id="2">02. Acknowledgments
* Our collision checker in the multicopter (collision-resilient drone) example are from UC Berkeley's HiPeRLab publication on [Exploiting collisions for sampling-based multicopter motion planning](https://doi.org/10.48550/arXiv.2011.04091). Tools in the CommonMath and Quartic directories are from UC Berkeley's HiPeRLab's [agri-fly repository](https://github.com/muellerlab/agri-fly). 

## <span id="3">03. License

The source code is released under [MIT](https://www.mit.edu/~amini/LICENSE.md) license.

## <span id="4">04. Maintenance

Feel free to contact us if you have any question.
