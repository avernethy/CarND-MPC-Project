#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.05;

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
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Differentiation: CppAD::pow(x, 2);
    // Declarations: CppAD<double

    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // state = 6 elements
  // actuators = 2 element vector
  // time steps = 25
  // -> 6 * 25 + 2 * 24 = 198
  size_t n_vars = 198;
  // TODO: Set the number of constraints
  size_t n_constraints = 7; //use the example code for now

  //state is the same as the x0 vector in the tutorial

  /*  0. Set N and dt. See above
        1. Fit the polynomial to the waypoints.
        2. Calculate initial cross track error and orientation error values.
        3. Define the components of the cost function (state, actuators, etc). You may use the methods previously discussed or make up something, up to you!
        4. Define the model constraints. These are the state update equations defined in the Vehicle Models module.*/

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = x0[3];
  double cte = state[4];
  double epsi = state[5];

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  int num_var = 25;
  int x_start = num_var;
  int y_start = x_start + num_var;
  int psi_start = y_start + num_var;
  int v_start = psi_start + num_var;
  int cte_start = v_start + num_var;
  int epsi_start = cte_start + num_var;
  int delta_start = epsi_start + num_var;
  int a_start = delta_start + num_var - 1;

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] =  0.436332;
  }
  for (int i = a_start; i < n_vars; i++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  /*Set N and dt.
  1. Fit the polynomial to the waypoints.
  2. Calculate initial cross track error and orientation error values.
  3. Define the components of the cost function (state, actuators, etc). You may use the methods previously discussed or make up something, up to you!
  4. Define the model constraints. These are the state update equations defined in the Vehicle Models module.*/
  return {};
}
