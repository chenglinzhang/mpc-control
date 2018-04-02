## 5. Model Predictive Control (MPC)

### Self-Driving Car Engineer Nanodegree Program - Term 2

In this project you'll implement Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. 


### Demo

The final program achieves successful self-driving around the track with a reference speed of 100 mph, as showing in the following:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/WA16us9gOgU/0.jpg)](https://www.youtube.com/watch?v=WA16us9gOgU)


[//]: # (Image References)
[image1]: ./images/cars_perspective.png "Car's Perspective"
[image2]: ./images/trajectory.png "Trajectory"
[image3]: ./images/mpc_model.png "MPC Model"
[image4]: ./images/kinematic.png "Kinematic Model"


### Data

The data input from the simulator is in JSON format. For example:
```
{"ptsx":[-158.9417,-146.6317,-134.6765,-126.4965,-108.5617,-95.20645],"ptsy":[-131.399,-141.329,-147.489,-150.849,-155.499,-157.609],"psi_unity":2.18962,"psi":5.664361,"x":-150.355,"y":-139.7533,"steering_angle":-0.03947658,"throttle":0.4531299,"speed":53.35338}
```

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. 
* `psi` (float) - The orientation of the vehicle in **radians**.
* `psi_unity` (float) - The orientation of the vehicle in **radians**. 
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.


### Data Transformation

The `(x,y)` coordinates of the waypoints are in world coordinate system. We will transform them into car coordinate system, to simplify the later model calculations, as illustrated in the following: 

<p align="center">
t_x<sub>n</sub> = x<sub>n</sub>cos(ψ) + y<sub>n</sub>sin(ψ)
<br/>
t_y<sub>n</sub> = y<sub>n</sub>cos(ψ) - x<sub>n</sub>sin(ψ)
</p>

![alt text][image1] <br/>
Courtesy to https://discourse-cdn-sjc3.com/udacity/uploads/default/original/4X/3/0/f/30f3d149c4365d9c395ed6103ecf993038b3d318.png


### MPC Model

MPC models the control problem of car trajectory following as an optimization problem. The solution to the optimization problem is the optimal tranjectory. In the following, the blue line illustrates the trajectory to follow, and the orange line shows an optimal trajectory for the car to move.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations, with two hyperparameters: N is the number of timesteps ahead, dt is how much time elapses between actuations. For example, if N were 20 and dt were 0.5, the car will look 10 seconds ahead for the prediction of an optimal trajectory. 

![alt text][image2] <br/>

Remember the car's control inputs are steering angle (δ) and throttle (a) in any given moment. The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by N:

<p align="center">
[δ<sub>1</sub>,a<sub>1</sub>,δ<sub>2</sub>,a<sub>2</sub>,...,δ<sub>N-1</sub>,a<sub>N-1</sub>]
</p>

A MPC model for car driving is defined in the following:

![alt text][image3] <br/>

where J is the cost function, with model constraints for x, y, ψ, v, cte, and eψ for each time point in prediction. Two range constraints for δ and a are added for practical driving. 

This makes the MPC model as a typical optimization problem, with mainly three components: 1) cost/objective function, 2) design variables, and 3) constraints (inequalities and equalities). Here the design variables are the vehicle states and control variables at every time step of the prediction. At time 0 we have (x, y, ψ, v, cte, eψ) as state input. 


### Current State

The data transformation on car's perspective simplifies the state input of x, y, and ψ to value 0. While v is given from input data, the cte (cross track error) and eψ values will need to be found by polynormal fitting of the previous transformed waypoints. The formulas are:

<p align="center">
cte = coeffs[0]
<br/>
eψ = -atan(coeffs[1])
</p>

where coeffs[0] and coeffs[1] are the first two coefficients of the fitted polymormal.


### Actuator Latency

In a real car, an actuation command (steering for example) will incur a latency before its effect is actually achieved. A realistic delay might be on the order of 100 milliseconds. MPC can model such dynamics by incorporating latency into the vehicle model. 

This project solves the latency problem at the time of state input: instead of feeding the "current state" to the optimizer, we feed a "predicated state" after the latency into the optimizer. The state prediction is performed in Global Kinematic Model:

![alt text][image4] <br/>


### Solver

The goal of an optimizer is to find the minimum of the cost/objective function by varying the values of the design variables while making sure all the constraints are satisfied. This is achieved in a C++ solver implementation in the files `MPC.h` and `MPC.cpp`. They again depends on a C++ `Ipopt` library for large-scale nonlinear optimization of continuous systems, and `CppAD`, a C++ library for automatic differentiation. 


### Parameter Tuning

First of all, I performed manual tuning on the weights to the cost funtion. Many weight combinations will work. However, I experiened most problems from sharp curves, where the car will go off the track when driving in higher reference speed. I ended up adding large weights to sequential actuations:
```
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 50000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
For N and d, I started with N=25, and d=0.25 and experiened an over-estimation problem. The car tried to take a "short cut" on sharp curves and ended up in lakes. I set it to N=10 and d=0.1 and it follows the track well with a reference speed of 100 mph.


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Code Style

Follows [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as far as possible.

