# **CarND Term 2 Project 2 Writeup** 
# **Unscented Kalman Filter** 

#### This repository contains a c++ implementation of an Model Predictive Control project. The repository is structured so that it works together with the [simulation environment provided by Udacity](https://github.com/udacity/self-driving-car-sim/releases). (The link between the environment and the code is via [uWebSocketIO](https://github.com/uNetworking/uWebSockets), and already provided by Udacity). 


## without latency and speed 40 actually the simulation runs as well
---

## Important scripts

This writeup will give an overview of the extended kalman filter implementation by going through 3 important scripts:

#### [1. main.cpp](#1.-main.cpp),
#### [2. tools.cpp](#2.-tools.cpp), and
#### [3. ukf.cpp](#3.-ukf.cpp).

### 1. main.cpp

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



No latency: stable until reference 85. resulting speed around 81.5
Latency ref speed 85 -> stable for one lap, but starts overshooting










### 2. tools.cpp

    VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,const vector<VectorXd> &ground_truth)

### 3. ukf.cpp



## Conclusion



## Note

