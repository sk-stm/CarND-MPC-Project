#ifndef MPC_H
#define MPC_H

#include "Polynome.h"
#include "Visualization.h"

#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

#include <tuple>
#include <vector>


using namespace std;

// some CPPAD related typedef
typedef CPPAD_TESTVECTOR(double) Dvector;

struct VehicleState {
  VehicleState() {}
  VehicleState(double x, double y, double psi, double v): x(x), y(y), psi(psi), v(v) {}
  double x {0};
  double y {0};
  double psi {0};
  double v {0};
};

class MPC {
  
public:
  MPC();

  virtual ~MPC();

  tuple<double, double> Calculate(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y, double x, double y, double psi, double v);
  
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
   * Return the first actuatotions.
   **/ 
  Dvector solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  /**
   * 
   **/ 
  void transformWaypoints(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y);
  
  /**
   * 
   **/
  void calcReferenceLine();
  
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
  Visualizer visualizer_;

  //! whether to plot the MPC trajectory
  bool doVisualize_{true};
};

#endif /* MPC_H */
