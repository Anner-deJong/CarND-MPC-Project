# **CarND Term 2 Project 5 Writeup** 
# **Model Predictive Control** 

#### This repository contains a c++ implementation of a Model Predictive Control project. The repository is structured so that it works together with the [simulation environment provided by Udacity](https://github.com/udacity/self-driving-car-sim/releases). (The link between the environment and the code is via [uWebSocketIO](https://github.com/uNetworking/uWebSockets), and already provided by Udacity). 


---

## Contents

This writeup will first introduce the two main .cpp files, and then discuss the overal implementation in terms of tuning and results.

#### [1. MPC.cpp](#1.-MPC.cpp),
##### [1.1 Variables](#1.1-Variables),
##### [1.2 Constraints](#1.2-Constraints),
##### [1.2 Cost function](#1.2-Cost-function),
#### [2. main.cpp](#2.-main.cpp),
##### [2.1 latency](#2.1-latency),
#### [3. Tuning](#3.-Tuning), and
#### [4. Results](#4.-Results).


### 1. MPC.cpp

Summarized, Model Predictive Control (MPC) is a form of continuous control, which, at each time step, solves a (non-linear) optimization problem several timesteps ahead into the future. The optimization problem's control output of the initial timestep is send back to the system, and the rest of the control outputs (for all timesteps into the future) are discarded. This process is repeated at each timestep. <br>
Actually, the main.cpp file implements this overall, repeating, MPC architecture. The MPC.cpp file merely implements the optimization problem that is called at each timestep from the main.cpp file. This optimization problem is a non-linear problem (NLP) that tries to drive the car safely around the track. The problem is solved with the c++ ipopt library. _How_ exactly it is solved is not further discussed here, only _what_ is solved. The key elements of the problem are variables, constraints and a cost function. The cost function is minimized (=optimizing), subject to the constraints depending on the variables.

#### 1.1 Variables

The variables are represented by the vehicle's state and control inputs at each time step. The state consist of:
* `x`,    the x location in [m],
* `y`,    the y location in [m],
* `psi`,  the vehicle's heading in [radians],
* `v`,    the vehicle's speed in [m/s],
* `cte`,  the cross track error (a measure of how far the car is away from the desired position), and
* `epsi`, the psi error (a measure of how far the car is turned away from the desired heading angle).

The control consists of:
* `delta`, the current steering angle command, and
* `a`,     the current acceleration command.

The NLP solves for a user-defined `N` timesteps into the future. Each timestep has a state, and for all but the last timestep there is a control input (as the control at the last timestep only influences the state at the last timestep+1, which is not accounted for). This results in `6*N + 2*(N-1)` variables.

#### 1.2 Constraints

The constraints represent how variables are related, and are implemented by means of a vehicle model. The vehicle model used here is the following:

    x_t+1    = x_t + v_t * cos(psi_t) * dt
    y_t+1    = y_t + v_t * sin(psi_t) * dt
    psi_t+1  = psi_t + v_t / Lf * delta_t * dt
    v_t+1    = v_t + a_t * dt
    cte_t+1  = y_t - y_gt_t + sin(epsi_t) * dt
    epsi_t+1 = psi_t - atan(y_gt_d_t) + v_t / Lf * delta_t * dt
    
where:
* `.._t` is a variable at timestep t,
* `.._t+1` is a variable at timestep t+1,
* `dt` is the user-defined time elapsed between timesteps
* `Lf` is a vehicle dependend parameter, relating steering commands and actual car angle changes
* `y_gt` is the ground truth / desired y position for `x`
* `y_gt_d_t` is the derivative of at the ground truth / desired y position for `x` (with atan this becomes the desired heading angle)

##### 1.2.1 Constraints implementation

In NLP's, it is common to rewrite constraints in order to have a variable that should equal 0. For the ipopt implementation, they are furthermore stored in one long vector `fg`. As such, the `x_t+1    = x_t (..)` equation above is implemented as (with `t` in the third row `=1`:
    
        x_1                  = x_0 + v_0 * cos(psi_0) * dt
    ->  constraint[x1]       = variable(x1) - (x0 + v0 * cos(psi0) * dt);
    ->  fg[1 + t + x_offset] = x1           - (x0 + v0 * CppAD::cos(psi0) * dt);

The +1 index for `fg` is because `fg` stores all the costs at index 0. The `x_offset` is required as `fg` contains all constraints for all variables and timesteps. The `CppAD` library functionality is also required for the ipopt implementation. 

Besides relating all variables between timesteps, the variables in the very first timestep should be made equal to the actual state of the car at that moment. For the x location this becomes:

    fg[1 + x_offset] = vars[x_offset];


#### 1.3 Cost function

All cost is added together to `fg[0]`. The cost function is split in three sections.

State cost:

    for (int t = 1; t < N; ++t) {
      fg[0] += CppAD::pow(v - v_ref, 2); // error in speed. v_ref is the reference/desired speed
      fg[0] += CppAD::pow(cte,       2); // cross track error
      fg[0] += CppAD::pow(epsi,      2); // heading angle error
    }

Direct actuator cost, penalizing strong actuater values:

    for (int t = 0; t < (N-1); ++t) {
      fg[0] += CppAD::pow(delta, 2); // try to prevent excessive steering angles
      // fg[0] += CppAD::pow(a,     2); // try to prevent excessive acceleration values
    }

And temporal actuator cost, causing smoother control over time:

    for (int t = 0; t < (N-2); ++t) {
      fg[0] += 10000 * CppAD::pow(delta1 - delta0, 2); // penalize more for steering command jerks
      fg[0] +=    50 * CppAD::pow(a1     - a0    , 2); // penalize more for acceleration jerks
    }

Each cost entry can be implemented with its own weight factor. Tuning these is discussed in [3. Tuning](#3.-Tuning).


### 2. main.cpp

The main script is taking care of the communication with the simulator (not further discussed here), and of implementing the actual MPC. A few constant reference parameters are defined at the top:

    double deg2rad(double x) { return x * M_PI / 180.; } // helper function
    constexpr double LATENCY           = 0.1;            // latency adjustment for MPC calculation, in seconds
    constexpr int    SIM_SLEEP_LATENCY = 100;            // simulator latency in miliseconds
    constexpr double V_REF             = 60. / 2.237;    // mph to mps
    constexpr double Lf                = 2.67;           // Also defined in MPC.cpp, prevent declaring this twice!!
    
Although `LATENCY` and `SIM_SLEEP_LATENCY` can be chosen differently, that should only be used for debugging. When there is an actual simulator latency, the MPC calculation adjustment should take this into account.

The MPC now works in 5 steps, before which the current state should be received from the simulator (in json package `j`):
    
    // get the state from the simulator
    vector<double> ptsx = j[1]["ptsx"];              // These are the x coordinates of the ground truth path
    vector<double> ptsy = j[1]["ptsy"];              // These are the y coordinates of the ground truth path
    double px    = j[1]["x"];
    double py    = j[1]["y"];
    double psi   = j[1]["psi"];
    double v     = double(j[1]["speed"]) / 2.237;    // convert from mph to m/s
    double delta = - double(j[1]["steering_angle"]); // minus because simulator has inversed sign for steering angles
    double a     = j[1]["throttle"];
    
STEP 1 - update car position for latency. This uses the same vehicle model as described in [1.2 Constraints](#1.2-Constraints). N.b. LATENCY here is the _expected_ time delay between sending a control command and the car actually performing said command. 
    
    double x_lat    = px  + v * std::cos(psi) * LATENCY;
    double y_lat    = py  + v * std::sin(psi) * LATENCY;
    double psi_lat  = psi + v / Lf * delta * LATENCY;
    double v_lat    = v   + a * LATENCY;
    
STEP 2 - map global ground truth path coordinates to local. As the ground truth coordinates are given in a global map coordinate system, yet the MPC is run in a local, car's perspective coordinate system, a tranformation of the ground truth coordinates is necessary.

    Eigen::VectorXd local_x(ptsx.size());
    Eigen::VectorXd local_y(ptsx.size());
    
    for (int i = 0; i < ptsx.size(); ++i) {
      double x_coord = ptsx[i] - x_lat;
      double y_coord = ptsy[i] - y_lat;
      local_x[i] =   std::cos(psi_lat) * x_coord + std::sin(psi_lat) * y_coord;
      local_y[i] = - std::sin(psi_lat) * x_coord + std::cos(psi_lat) * y_coord;
    }

STEP 3 - fit polynomial

    Eigen::VectorXd coeffs = polyfit(local_x, local_y, 3);

          // STEP 4 - create 6 dim state (calculate cte, epsi)
          // local state variables. The car is now at the origin so x, y, and psi are 0

          double x_loc = 0.; double y_loc = 0.; double psi_loc = 0.; double v_loc = v_lat;
          double cte_loc  = - polyeval(coeffs, x_loc);
          double epsi_loc = - std::atan(polyeval_derivative(coeffs, x_loc));
          
          Eigen::VectorXd state(6);
          state << x_loc, y_loc, psi_loc, v_loc, cte_loc, epsi_loc;

          // STEP 5 - Run solver
          vector<double> results = mpc.Solve(state, coeffs);


### 3. Tuning

cost tuning
pow(a, 2) is turned off
N tuning
dt tuning
speed tuning



### 4. Results

No latency: stable until reference 85. resulting speed around 81.5
Latency ref speed 85 -> stable for one lap, but starts overshooting

1.    60 mph marginally (un)stable
      fg[0] += 1000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=    1 * CppAD::pow(a1     - a0    , 2);

2.    60 mph unstable
      fg[0] += 1000 * CppAD::pow(delta1 - delta0, 2); -> 50 mph
      fg[0] +=  100 * CppAD::pow(a1     - a0    , 2);

3.    60 mph marginally (un)stable
      fg[0] += 4000 * CppAD::pow(delta1 - delta0, 2); // penalize more for delta jerks
      fg[0] +=  100 * CppAD::pow(a1     - a0    , 2);

4.    60 mph marginally (un)stable
      fg[0] += 3000 * CppAD::pow(delta1 - delta0, 2); // penalize more for delta jerks
      fg[0] +=   10 * CppAD::pow(a1     - a0    , 2);

5.    60 mph marginally (un)stable
      fg[0] += 5000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=    1 * CppAD::pow(a1     - a0    , 2);

6.    55 mph stable
      fg[0] += 5000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=    1 * CppAD::pow(a1     - a0    , 2);

7.    55 mph unstable
      fg[0] +=  100 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=    1 * CppAD::pow(a1     - a0    , 2);

8.    55 mph unstable
      fg[0] += 1000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=    1 * CppAD::pow(a1     - a0    , 2);

9.    55 mph stable, but still breaking
      fg[0] += 5000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   10 * CppAD::pow(a1     - a0    , 2);

9.    55 mph stable, no breaking
      fg[0] += 5000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);

10.   60 mph marginally (un)stable
      fg[0] += 5000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);

11.   60 mph marginally stable
      fg[0] += 7000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);

12.   60 mph stable
      fg[0] += 10000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);

13.   65 mph unstable
      fg[0] += 10000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);

14.   65 mph unstable
      fg[0] += 20000 * CppAD::pow(delta1 - delta0, 2);
      fg[0] +=   50 * CppAD::pow(a1     - a0    , 2);


### Note

