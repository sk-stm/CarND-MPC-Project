#ifndef MPC_H
#define MPC_H

#include "Polynome.h"

#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct VehicleState {
  VehicleState() {}
  VehicleState(double x, double y, double psi, double v): x(x), y(y), psi(psi), v(v) { }
  double x {0};
  double y {0};
  double psi {0};
  double v {0};
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  tuple<double, double> Calculate(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y, double x, double y, double psi, double v);
  
  tuple<vector<double>, vector<double>> GetLocalWaypoints();
  tuple<vector<double>, vector<double>> GetLocalMPCPoints();
  
 protected:
  void transformWaypoints(vector<double> const & waypoints_global_x, vector<double> const & waypoints_global_y);
  void calcReferenceLine();
  
  vector<double> waypoints_local_x_;
  vector<double> waypoints_local_y_;
  VehicleState vehicleState_;
  Polynome referenceLine_;
  
};

#endif /* MPC_H */
