#pragma once

#include <string>

using namespace std;


//  IPOpt Options
/*********************************************/

string getOptions() {
  // options for IPOPT solver
  std::string ipopt_options;
  ipopt_options += "Integer print_level   0\n";
  ipopt_options += "Sparse  true          forward\n";
  ipopt_options += "Sparse  true          reverse\n";
  ipopt_options += "Numeric max_cpu_time  0.5\n";

  return ipopt_options;
}


//  MPC constants
/*********************************************/

//! number of trajectory points
const size_t N = 10;
//! time step between trajectory points
const double dt = 0.2;
// steering ration, tunes by Udacity
const double Lf = 2.67;

//! target velocity (m/s)
const double ref_v = 20;


//  IPOpt variable indices
/*********************************************/
const size_t x_start     = 0;
const size_t y_start     = x_start + N;
const size_t psi_start   = y_start + N;
const size_t v_start     = psi_start + N;
const size_t cte_start   = v_start + N;
const size_t epsi_start  = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start     = delta_start + N - 1;