#include "MPC.h"

#include "constants.h"
#include "Visualization.h"

#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Geometry"
#include "json.hpp"


#include <chrono>

using CppAD::AD;


class FG_eval {
protected:
  //! polynomial coefficients
  Eigen::VectorXd coeffs_;
  //! weights for some of the cost terms
  map<string, double> const & weights_;


 public:
  FG_eval(Eigen::VectorXd const & coeffs, map<string, double> const & weights) : 
    coeffs_(coeffs), 
    weights_(weights) { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    fg[0] = 0;
    
    
    //
    // Reference State Cost

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += weights_.at("cte")  * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += weights_.at("epsi") * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += weights_.at("v")    * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weights_.at("steering") * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weights_.at("throttle") * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weights_.at("steering_velocity") * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weights_.at("jerk")              * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Minimize the centripetal acceleration (i.e. brake for curves) ..
    // .. calculate ca from steering angle and velocity
    // see: https://discussions.udacity.com/t/tuning-n-and-dt-can-it-be-dynamic/379832)
    //      https://nabinsharma.wordpress.com/2014/01/02/kinematics-of-a-robot-bicycle-model/
    for (int t = 0; t < N - 1; t++) {
      // absolute steering angle + eps (such that division by zero is avoided)
      // could this also be solved with conditional expressions? https://www.coin-or.org/CppAD/Doc/cond_exp.cpp.htm
      auto delta = CppAD::abs(vars[delta_start + t]) + 1e-6;
      // the turning radius: r = L / tan(delta)
      auto r = Lf / CppAD::tan(delta);
      // centripetal acceleration: a_r = v^2 / r;
      auto v = vars[v_start + t];
      auto a_r = CppAD::pow(v, 2) / r;
      fg[0] += weights_.at("ca") * CppAD::pow(a_r, 2);

      // simple version:
      //fg[0] += weights_.at("ca") * CppAD::pow(v * delta, 2);
    }


    //
    //  Initial constraints

    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {

      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // TODO: this depends on poly_order==3!! 
      // .. f(x) = coeffs_[3]x^3 + coeffs_[2]x^2 + coeffs_[1]*x + coeffs_[0]
      AD<double> f0 = coeffs_[3]*CppAD::pow(x0, 3) + coeffs_[2]*CppAD::pow(x0, 2) + coeffs_[1]*x0 + coeffs_[0];
      // .. f'(x) = 3*coeffs_[3]x^2 + 2*coeffs_[2]x + coeffs_[1]
      AD<double> psides0 = CppAD::atan( 3*coeffs_[3]*CppAD::pow(x0, 2) + 2*coeffs_[2]*x0 + coeffs_[1] );

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  visualizer_                   = new Visualizer();

  // parse input params
  auto params = nlohmann::json::parse(std::ifstream("params.json"));
  dt                            = params["dt"];
  latency                       = params["latency"];
  N                             = params["N"];
  ref_v                         = params["ref_v"];
  weights_["steering_velocity"] = params["weights"]["steering_velocity"];
  weights_["jerk"]              = params["weights"]["jerk"];
  weights_["cte"]               = params["weights"]["cte"];
  weights_["epsi"]              = params["weights"]["epsi"];
  weights_["v"]                 = params["weights"]["v"];
  weights_["steering"]          = params["weights"]["steering"];
  weights_["throttle"]          = params["weights"]["throttle"];
  weights_["ca"]                = params["weights"]["ca"];
}
MPC::~MPC() {}

Dvector MPC::solve(Eigen::VectorXd state) {
  // IPOpt error state
  bool ok = true;
  // number of independent variables: N timesteps == N - 1 actuations
  static size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  static size_t n_constraints = N * 6 + (N - 1) * 2;

  // start variables
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];


  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/deceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;



  // object that computes objective and constraints
  FG_eval fg_eval(referenceLine_.getCoeffs(), weights_);
  

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      getOptions(), vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if(not ok) {
    cerr << "Solver not okay :/" << endl;
    //throw runtime_error("Solver not okay :/");
  }

  // Cost
  auto cost = solution.obj_value;
  std::cout << "-- cost " << cost << std::endl;

  // return the first actuator values
  return solution.x;
}


tuple<double, double> MPC::Calculate(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y, double px, double py, double psi, double v, double throttle, double steering) {

  // set new vehicle state
  vehicleState_ = VehicleState(px, py, psi, v, throttle, steering);

  // forward simulate the vehicle to account for latency
  forwardSimulate(latency);

  // fit a polynome to the local waypoints
  calcReferenceLine(waypoints_global_x, waypoints_global_y);

  // get current orientation and cross-track error
  // .. the reference line is relative to the vehicle (which is therefore at (0,0), so eval(0) gives the cte
  double cte = referenceLine_.eval(0);
  // .. similar, our vehicle's rotation is zero in this frame, so the orientation error is the gradient orientation (i.e. slope direction) at 0
  double epsi = -atan(referenceLine_.derivative(0));

  // gather start state of the MPC
  Eigen::VectorXd startState(7);
  startState << 0, 0, 0, vehicleState_.v, cte, epsi, vehicleState_.steering;
  
  // run the optimization
  auto start = std::chrono::high_resolution_clock::now();
  mpcSolution_ = solve(startState);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> duration = end - start;
  std::cout << "-- time: " << std::setprecision(2) << duration.count() << std::endl;

  
  // return values
  double steer_value             = mpcSolution_[delta_start];
  double throttle_value          = mpcSolution_[a_start];

  // visualization
  if(doVisualize_) {
    visualizer_->setData(mpcSolution_, startState);
  }

  return std::make_pair(steer_value, throttle_value);
}

void MPC::forwardSimulate(double dTime) {
  vehicleState_.x   = vehicleState_.x + vehicleState_.v * cos(vehicleState_.psi) * dTime;
  vehicleState_.y   = vehicleState_.y + vehicleState_.v * sin(vehicleState_.psi) * dTime;
  vehicleState_.psi = vehicleState_.psi + vehicleState_.v / Lf * tan(vehicleState_.steering) * dTime;

  // unfortunately the simulator reports wrong throttle/acceleration values
  // see: https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/88
  vehicleState_.v = vehicleState_.v + vehicleState_.throttle * dTime;
}

void MPC::transformWaypoints(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y) {
  assert(waypoints_global_x.size() == waypoints_global_y.size() && "input vectors have to have the same size!");

  // resize lists (does not clear)
  waypoints_local_x_.resize(waypoints_global_x.size());
  waypoints_local_y_.resize(waypoints_global_y.size());

  // matrix transforming the waypoints from the global frame to the local one
  Eigen::Rotation2D<double> rot(vehicleState_.psi);
  Eigen::Translation2d trans(vehicleState_.x, vehicleState_.y);
  Eigen::Transform<double, 2, Eigen::Affine> tf = trans * rot;
  tf = tf.inverse();

  // transform points
  for(size_t i = 0; i < waypoints_global_x.size(); ++i) {
    Eigen::Vector2d wp(waypoints_global_x[i], waypoints_global_y[i]);
    Eigen::Vector2d wpLocal = tf * wp;
    waypoints_local_x_[i] = wpLocal[0];
    waypoints_local_y_[i] = wpLocal[1];
  }
}


void MPC::calcReferenceLine(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y) {

  // transform waypoints from global to local coordinate system
  transformWaypoints(waypoints_global_x, waypoints_global_y);

  // fit polynome to local waypoints
  referenceLine_ = Polynome(waypoints_local_x_, waypoints_local_y_, 3);
}

tuple<vector<double>, vector<double>> MPC::GetLocalWaypoints() {
  vector<double> x_vals, y_vals;
  
  if(waypoints_local_x_.size() == 0) {
    return std::make_pair(x_vals, y_vals);
  }

  const double dx = 2;
  for(double x = waypoints_local_x_.front(); x <= waypoints_local_x_.back(); x += dx) {
    double y = referenceLine_.eval(x);
    x_vals.push_back(x);
    y_vals.push_back(y);
  }

  return std::make_pair(x_vals, y_vals);
}

tuple<vector<double>, vector<double>> MPC::GetLocalMPCPoints() {
  vector<double> x_vals, y_vals;
  
  if(mpcSolution_.size() == 0) {
    return std::make_pair(x_vals, y_vals);;
  }
  
  for(double i = 0; i < N; i++) {
    x_vals.push_back(mpcSolution_[x_start + i]);
    y_vals.push_back(mpcSolution_[y_start + i]);
  }

  return std::make_pair(x_vals, y_vals);
}

