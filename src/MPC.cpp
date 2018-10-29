#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// degree 2 radian utility function
namespace {
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }
}

// TODO: Set the timestep length, duration and reference speed
size_t N     = 25;
double dt    = 1.25/N;

// define index offsets
int x_offset     = 0 * N;
int y_offset     = 1 * N;
int psi_offset   = 2 * N;
int v_offset     = 3 * N;
int cte_offset   = 4 * N;
int epsi_offset  = 5 * N;
int delta_offset = 6 * N;
int a_offset     = 7 * N - 1; // cause there are (N-1) delta entries

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double v_ref;
  FG_eval(Eigen::VectorXd coeffs, double v_reference) {
    this->coeffs = coeffs;
    this->v_ref  = v_reference;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // ALWAYS add 1 when indexing fg, as fg prepends the cost at index 0
    
    /***************************************/
    /**************** COSTS ****************/
    
    // initialize cost ( fg[0] ) as zero
    fg[0] = 0;

    // state cost
    for (int t = 1; t < N; ++t) {

      // get parameters
      AD<double> v    = vars[t +    v_offset];
      AD<double> cte  = vars[t +  cte_offset];
      AD<double> epsi = vars[t + epsi_offset];

      // add cost
      fg[0] += CppAD::pow(v - v_ref, 2);
      fg[0] += CppAD::pow(cte,       2);
      fg[0] += CppAD::pow(epsi,      2);
    }

    // direct actuator cost; penalize strong actuater values
    for (int t = 0; t < (N-1); ++t) {

      // get parameters
      AD<double> delta = vars[t + delta_offset];
      AD<double> a     = vars[t +     a_offset];

      // add cost
      fg[0] += CppAD::pow(delta, 2);
      // fg[0] += CppAD::pow(a,     2);
    }

    // temporal actuator cost (smoothness cost)
    for (int t = 0; t < (N-2); ++t) {

      // get parameters at timestep t
      AD<double> delta0 = vars[t + 0 + delta_offset];
      AD<double> a0     = vars[t + 0 +     a_offset];
      
      // get parameters at timestep t + 1
      AD<double> delta1 = vars[t + 1 + delta_offset];
      AD<double> a1     = vars[t + 1 +     a_offset];

      // add cost
      fg[0] += 100 * CppAD::pow(delta1 - delta0, 2); // penalize more for delta jerks
      fg[0] +=   1 * CppAD::pow(a1     - a0    , 2);
    }

    /***************************************/
    /************* CONSTRAINTS *************/

    // initial state constraints
    fg[1 +     x_offset] = vars[    x_offset];
    fg[1 +     y_offset] = vars[    y_offset];
    fg[1 +   psi_offset] = vars[  psi_offset];
    fg[1 +     v_offset] = vars[    v_offset];
    fg[1 +   cte_offset] = vars[  cte_offset];
    fg[1 +  epsi_offset] = vars[ epsi_offset];
    
    // all subsequent state constraints
    for (int t = 1; t < N; ++t) {

       // get parameters at timestep t - 1
      AD<double> x0     = vars[t - 1 +     x_offset];
      AD<double> y0     = vars[t - 1 +     y_offset];
      AD<double> psi0   = vars[t - 1 +   psi_offset];
      AD<double> v0     = vars[t - 1 +     v_offset];
      AD<double> cte0   = vars[t - 1 +   cte_offset];
      AD<double> epsi0  = vars[t - 1 +  epsi_offset];
      AD<double> delta0 = vars[t - 1 + delta_offset];
      AD<double> a0     = vars[t - 1 +     a_offset];
      
      // get parameters at timestep t
      AD<double> x1     = vars[t +     x_offset];
      AD<double> y1     = vars[t +     y_offset];
      AD<double> psi1   = vars[t +   psi_offset];
      AD<double> v1     = vars[t +     v_offset];
      AD<double> cte1   = vars[t +   cte_offset];
      AD<double> epsi1  = vars[t +  epsi_offset];

      // calculate y and y' according to the polyfit
      AD<double> y0_gt   = coeffs[0];
      AD<double> y0_gt_d = 0.0;

      for (int i = 1; i < coeffs.size(); ++i) {
        y0_gt   +=     coeffs[i] * CppAD::pow(x0, i);
        y0_gt_d += i * coeffs[i] * CppAD::pow(x0, i-1);
      }

      // add constraints
      fg[1 + t +    x_offset] = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + t +    y_offset] = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      // THIS SHOULD BECOME MINUS: ACCORDING TO INVERSED ANGLES. SEE PROJECT DESCRIPTION
      fg[1 + t +  psi_offset] = psi1  - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + t +    v_offset] = v1    - (v0 + a0 * dt);
      fg[1 + t +  cte_offset] = cte1  - (y0 - y0_gt + CppAD::sin(epsi0) * dt);
      // THIS LINE AS WELL? THIS SHOULD BECOME MINUS: ACCORDING TO INVERSED ANGLES. SEE PROJECT DESCRIPTION
      fg[1 + t + epsi_offset] = epsi1 - (psi0 - CppAD::atan(y0_gt_d) + v0 / Lf * delta0 * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(int VERBOSE_LEVEL, double V_REF) {
  this->verbose     = VERBOSE_LEVEL;
  this->v_reference = V_REF;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  // size_t i; // <-- What is this unused declaration??
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_state = state.size();

  /***************************************/
  /************** VARIABLES **************/

  // size
  size_t n_vars = N * n_state + (N-1) * 2;

  // variable vectors
  Dvector vars(n_vars); // Initial values of the independent variables
  Dvector vars_lowerbound(n_vars); // vector for variables' lower limit
  Dvector vars_upperbound(n_vars); // vector for variables' upper limit
  
  // variable initial values should be 0, except for the initial state's
  // state respectively: x loc, y loc, psi, v, cte, e-psi
  // variable limits only in place for the actuator values

  // initialize all state variables with 0 and no bounds
  for (int i = 0; i < N * n_state; ++i) {
    vars[i] = 0;
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // adjust the initial state variable values
  for (int i = 0; i < n_state; ++i) {
    vars[i * N] = state[i];
  }

  // steering actuator variables: delta. Limit set to 25 degrees
  for (int i = N * n_state; i < N * n_state + (N-1); ++i) {
    vars[i] = 0;
    vars_lowerbound[i] = deg2rad(-25);
    vars_upperbound[i] = deg2rad(25);
  }
  
  // accelaration actuator variables: a. Simulator limit [-1, 1]
  for (int i = N * n_state + (N-1); i < n_vars; ++i) {
    vars[i] = 0;
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }


  /***************************************/
  /************* CONSTRAINTS *************/
  // constraint bounds should all be 0, except for initial state
  
  // size
  size_t n_constraints = N * state.size();

  // contstraint vectors
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  // initialize all constraint bounds with 0
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // adjust the initial state vars constraint bounds
  for (int i = 0; i < n_state; i++) {
    constraints_lowerbound[i * N] = state[i];
    constraints_upperbound[i * N] = state[i];
  }


  /***************************************/
  /****** INITIALIZE AND RUN SOLVER ******/

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, v_reference);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  if (verbose >= 1) {std::cout << "Cost " << cost << std::endl;}

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  // return the first most controls, as well as the predicted coordinates.
  // NB coordinates are arrange x0, y0, x1, y1, etc.. instead of x0, x1, .., y0, y1, ..

  std::vector<double> results = {solution.x[delta_offset], solution.x[a_offset]};

  for (int i=0; i<N; ++i) {
    results.push_back(solution.x[x_offset + i]); // add x coordinate
    results.push_back(solution.x[y_offset + i]); // add y coordinate
  }
  
  return results;

}






