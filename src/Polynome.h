#pragma once

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <vector>

using namespace std;


class Polynome {
public:
    Polynome() = default;

    /**
     * Constructor fitting a polynome to the given values.
     **/
    Polynome(vector<double> const & points_x, vector<double> const & points_y, int order = 3) {
         // get approximate length, resp. max x value (assumed the last value is the biggest one)
         max_x = points_x.back();
         order_ = order;

        // std to Eigen conversion (no copy)
        const Eigen::Map<const Eigen::VectorXd> xvals(points_x.data(), points_x.size());
        const Eigen::Map<const Eigen::VectorXd> yvals(points_y.data(), points_y.size());

        // poly fitting
        fit(xvals, yvals, order);
    }

    /**
     * Evaluate the polynome at x.
     **/ 
    double eval(double x) {
        double result = 0.0;

        for (int i = 0; i < coeffs_.size(); i++) {
            result += coeffs_[i] * pow(x, i);
        }

        return result;
    }

    /**
     * Evaluate the derivative of the polynome at x.
     **/ 
    double derivative(double x) {
      double result = 0.0;

      for (int i = 1; i < coeffs_.size(); i++) {
          result += i*coeffs_[i] * pow(x, i-1);
      }

      return result;
    }

    /**
     * Returns the coefficients of the polynome.
     **/ 
    Eigen::VectorXd getCoeffs() {
      return coeffs_;
    }

  
private:
  /**
   * Fit a polynomial.
   * 
   * @see https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
   **/ 
  void fit(Eigen::VectorXd const & xvals, Eigen::VectorXd const & yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    
    Eigen::MatrixXd A(xvals.size(), order + 1);
  
    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }
  
    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }
  
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    coeffs_ = result;
  }

  //! the polynomials coefficients (inverse order)
  Eigen::VectorXd coeffs_;
  //! the order of the polynomial
  int order_ {0};
  //! the maximum x-value of the input points (maps to a "valid length"; min_x is assumed to be <= 0)
  double max_x;
};