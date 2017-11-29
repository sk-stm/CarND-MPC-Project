#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include "Polynome.h"

#include <cppad/cppad.hpp>

#include <map>
#include <tuple>
#include <vector>

using namespace std;

// some CPPAD related typedef
typedef CPPAD_TESTVECTOR(double) Dvector;
// fwd decl
class Visualizer;

struct VehicleState {
  VehicleState() {}
  VehicleState(double x, double y, double psi, double v, double throttle, double steering): x(x), y(y), psi(psi), v(v), throttle(throttle), steering(steering) {}
  double x {0};
  double y {0};
  double psi {0};
  double v {0};
  double throttle {0};
  double steering {0};
};

class MPC {
  
public:
  MPC();

  virtual ~MPC();

  tuple<double, double> Calculate(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y, double x, double y, double psi, double v, double throttle, double steering);
  
  /**
   * 
   **/
  tuple<vector<double>, vector<double>> GetLocalWaypoints();

  /**
   * 
   **/
  tuple<vector<double>, vector<double>> GetLocalMPCPoints();
  
protected:
  /**
   * Solve the model given an initial state and polynomial coefficients.
   * Returns the actuation commands to be executed.
   **/ 
  Dvector solve(Eigen::VectorXd state);

  /**
   * Transforms the global waypoints to local onces (vehicle frame), and fits a polynome through them.
   **/ 
  void transformWaypoints(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y);
  
  /**
   * Builds the local reference line transforming global waypoints and polynome fitting through them.
   **/
  void calcReferenceLine(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y);

  /**
   * Forward simulates the vehicle state with the CTRV motion model.
   * @note Udacity presented a very simplified kinematic model:
   * @see https://discussions.udacity.com/t/small-angle-approximation-in-psi-prediction-equation/306208
   **/
  void forwardSimulate(double t);
  
  //! the x-values of the waypoints, transformed to be w.r.t. the vehicle frame
  vector<double> waypoints_local_x_;

  //! the y-values of the waypoints, transformed to be w.r.t. the vehicle frame
  vector<double> waypoints_local_y_;

  //! the current vehicle state
  VehicleState vehicleState_;

  //! the reference line (w.r.t. the vehicle frame)
  Polynome referenceLine_;

  //! the current MPC solution vector
  Dvector mpcSolution_;

  //! wrapper for 2D plotting
  Visualizer* visualizer_{nullptr};

  //! whether to plot the MPC trajectory
  bool doVisualize_{true};

  //! the weights for some cost terms
  map<string, double> weights_;
};

#endif /* MPC_H */
