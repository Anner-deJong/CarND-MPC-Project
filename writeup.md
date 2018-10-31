# **CarND Term 2 Project 5 Writeup** 
# **Model Predictive Control** 

#### This repository contains a c++ implementation of a Model Predictive Control project. The repository is structured so that it works together with the [simulation environment provided by Udacity](https://github.com/udacity/self-driving-car-sim/releases). (The link between the environment and the code is via [uWebSocketIO](https://github.com/uNetworking/uWebSockets), and already provided by Udacity). 


---

## Contents

This writeup will first introduce the two main .cpp files, and then discuss the overall implementation in terms of tuning and results.

[1. MPC.cpp](#1.-MPC.cpp), <br>
&nbsp;&nbsp;[1.1 Variables](#1.1-Variables), <br>
&nbsp;&nbsp;[1.2 Constraints](#1.2-Constraints), <br>
&nbsp;&nbsp;[1.3 Cost function](#1.3-Cost-function), <br>

[2. main.cpp](#2.-main.cpp), <br>
[3. Tuning](#3.-Tuning), and <br>
[4. Results](#4.-Results).


### 1. MPC.cpp

Summarized, Model Predictive Control (MPC) is a form of continuous control, which, at each time step, solves a (non-linear) optimization problem several timesteps ahead into the future. The optimization problem's control output of the initial timestep is send back to the system, and the rest of the control outputs (for all timesteps into the future) are discarded. This process is repeated at each timestep. <br>
Actually, the main.cpp file implements this overall, repeating, MPC architecture. The MPC.cpp file merely implements the optimization problem that is called at each timestep from the main.cpp file. This optimization problem is a non-linear problem (NLP) that tries to drive the car safely around the track. The problem is solved with the c++ ipopt library. _How_ exactly it is solved is not further discussed here, only _what_ is solved. The key elements of the problem are variables, constraints and a cost function. The cost function is minimized (=optimizing), subject to the constraints depending on the variables.

#### 1.1 Variables

The variables are represented by the vehicle's state and control inputs at each time step. The state consists of:
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
* `Lf` is a vehicle dependent parameter, relating steering commands and actual car angle changes
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

Direct actuator cost, penalizing strong actuator values:

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
    constexpr int    SIM_SLEEP_LATENCY = 100;            // simulator latency in milliseconds
    constexpr double V_REF             = 60. / 2.237;    // mph to mps
    constexpr double Lf                = 2.67;           // Also defined in MPC.cpp, prevent declaring this twice!!

There are several reasons why an autonomous driving system could incur a delay between being positioned in a state and actually performing the control output for said position. The causes can be digital (receiving current state values, calculating control output, sending resulting control outputs through the system, etc.) as well as mechanical (bending rods, dynamic frictions, backlash, etc.). In the current simulator setup, these latencies are perhaps negligible. As a model for a real system however, the MPC should be able to take care of latencies. As such, an active latency is introduced in the system by means of `SIM_SLEEP_LATENCY`. <br>
Although `LATENCY` and `SIM_SLEEP_LATENCY` can be chosen differently, that should only be done while debugging. During normal practice, when there is an actual (simulator) latency, the MPC calculation should take this into account.

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
    
STEP 2 - map global ground truth path coordinates to local. As the ground truth coordinates are given in a global map coordinate system, yet the MPC is run in a local, car's perspective coordinate system, a transformation of the ground truth coordinates is necessary.

    Eigen::VectorXd local_x(ptsx.size());
    Eigen::VectorXd local_y(ptsx.size());
    
    for (int i = 0; i < ptsx.size(); ++i) {
      double x_coord = ptsx[i] - x_lat;
      double y_coord = ptsy[i] - y_lat;
      local_x[i] =   std::cos(psi_lat) * x_coord + std::sin(psi_lat) * y_coord;
      local_y[i] = - std::sin(psi_lat) * x_coord + std::cos(psi_lat) * y_coord;
    }

STEP 3 - fit a polynomial based on the local ground truth coordinates. This is necessary to calculate cte and epsi for example.

    Eigen::VectorXd coeffs = polyfit(local_x, local_y, 3);

STEP 4 - create the 6-dim state. The car is now at the origin of the coordinate system, so `x`, `y`, and `psi` are 0. `v` = `v_lat`, and `cte` and `epsi` have to be calculated according the vehicle model's equations.

    double x_loc = 0.; double y_loc = 0.; double psi_loc = 0.; double v_loc = v_lat;
    double cte_loc  = - polyeval(coeffs, x_loc);
    double epsi_loc = - std::atan(polyeval_derivative(coeffs, x_loc));
    
    Eigen::VectorXd state(6);
    state << x_loc, y_loc, psi_loc, v_loc, cte_loc, epsi_loc;

STEP 5 - Run the NLP solver (the MPC.cpp file described above in [MPC.cpp](#1.-MPC.cpp))

    vector<double> results = mpc.Solve(state, coeffs);
    
`results` now contains the optimized first steering control value, first acceleration control value, and the predicted car trajectory according to all the `N` time steps.


### 3. Tuning

Several hyperparameters need to be tuned for the current MPC setup:
* `N = 25`, the amount of timesteps the NLP should solve for,
* `dt = 1.25/N = 0.05 [s]`, the time elapsed between these `N` timesteps,
* `V_REF = 26.82 [m/s]`, the reference speed for the vehicle
* weight parameters for each of the difference cost function additions

##### `N` and `dt`
For `N` and `dt` there is a trade-off: smaller `dt` allows for more accurate approximations of how the vehicle propagates in time, but with the same amount of `N` the total time span of the trajectory checked is smaller and might lead to errors later on. Increasing `N` allows an increase in accuracy by looking further ahead, or in combination with decreasing `dt` to have a more accurate horizon, but this comes at an increased computation cost. The current values of `25` and `1.25/N` (meaning the MPC will look 1.25 seconds ahead into the future) are not necessarily globally optimal. They do however seem to work well enough, so tuning time was instead spent on tuning the speed and cost.

##### Speed
Simply said, the quicker a system propagates over time, the faster and/or more robust a controller would have to be to make it stable. Choosing the speed therefore becomes more like a game: adjusting the other parameters, what is the maximum reference speed with which the vehicle can still stably drive around the track? With the current implementation this is around 60 miles/hour, or 26.82 meter/second. (The actual resulting speed is slightly lower).

##### Cost
Since the cost is simply an addition of different terms, they could each be given a weight indicating their relative importance. Trying out different values, it seems that increasing any of the cost terms except for the two temporal terms, makes the system perform worse and often make the vehicle crash. <br>
Without increasing the acceleration temporal smoothness weight to 50 (`fg[0] += weight * CppAD::pow(a1     - a0    , 2);`), the vehicle repeatedly keeps speeding up to the reference speed and vehemently breaking after it reaches it. Bumping the weight up to 50 results in a much smoother acceleration and deceleration, and a stable constant velocity. <br>
The steering command temporal smoothness weight of 10000 (`fg[0] += weight * CppAD::pow(delta1 - delta0, 2);`) seems almost ridiculously out of place. However, with a weight of 5000 the vehicle is unstable, and with 7000 the vehicle is barely stable, perhaps not even for several consecutive rounds. <br>
A weight of 20000 with a reference speed bumped up by 5 mph to 65 mph was also tried, but turned out to be unstable. Further experimenting might yield stable systems with higher reference velocities.


### 4. Results

As mentioned before, the current implementation accounts for a 0.1 second latency, and is able to keep the vehicle stable up to a reference velocity of 60 m/h, or 26.82 m/s. Already noticeable with this reference speed is a slight swinging around the reference trajectory. Somehow the system is only just able to dampen this, but fails to do so with higher velocities. The vehicle 

This might suggest adjusting the `N`, `dt` or cost structure to increase the damping of the system might render higher velocities stable.

Interestingly enough, although any reference speed above 60 mph renders the vehicle unstable, a reference speed of 65 is quite unstable, yet a reference speed of 85 is much less unstable, and often able to drive around a full round around the track before crashing.

Considering the latency, it _does_ deteriorate the robustness of the system. Without any latency, and not changing the cost weights, the vehicle is stable up to a reference speed of 85 mph (actual speed around 81.5 mph). This indicates that the current MPC adjustment for latency might not be optimal.

