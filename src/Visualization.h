#pragma once

#include "constants.h"
#include "matplotlibcpp.h"

#include <cppad/cppad.hpp>
#include <chrono>
#include <mutex>
#include <thread>

namespace plt = matplotlibcpp;
using namespace std;
typedef CPPAD_TESTVECTOR(double) Dvector;


class Visualizer {

public:
  Visualizer() {

    run_ = true;

    // setup the worker thread
    worker_ = std::thread([&](){
      while(run_) {
        if(dirty_) {
          dirty_ = false;
          plot();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  ~Visualizer() {
    run_ = false;
    worker_.join();
  }

  void setData(Dvector const & solution, Eigen::VectorXd vehicleState) {
    std::lock_guard<std::mutex> lock{mutex_};
    solutionData_ = solution;
    
    velocityData_.push_back(vehicleState[3]);
    if(velocityData_.size() > 4. / dt) {
      velocityData_.erase(velocityData_.begin());
    }

    steeringData_.push_back(vehicleState[6]);
    if(steeringData_.size() > 4. / dt) {
      steeringData_.erase(steeringData_.begin());
    }
    
    
    dirty_ = true;
  }

private:
  std::mutex mutex_;
  std::thread worker_;
  Dvector solutionData_;
  std::vector<float> velocityData_;
  std::vector<float> steeringData_;
  
  bool dirty_ {false};
  bool run_{false};


  void plot(){
    mutex_.lock();
    vector<float> steering;
    vector<float> throttle;
    vector<float> velocity;
    vector<float> psi;
    vector<float> cte;
    vector<float> epsi;

    // fill actuation data
    for(size_t i = 0; i < N-1; ++i) {
      steering.push_back(solutionData_[delta_start+i]);
      throttle.push_back(solutionData_[a_start+i]);
    }

    // fill trajectory data
    for(size_t i = 0; i < N; ++i) {
      velocity.push_back(solutionData_[v_start+i]);
      psi.push_back(solutionData_[psi_start+i]);
      cte.push_back(solutionData_[cte_start+i]);
      epsi.push_back(solutionData_[epsi_start+i]);
    }
    mutex_.unlock();



    //
    // actual plotting

    // clear previous plot
    plt::clf();
    
    plt::subplot(5, 1, 1);
    plt::named_plot("steering (rad)", steering);
    plt::named_plot("throttle (m/s)", throttle);
    plt::title("MPC Trajectory");
    plt::legend();

    plt::subplot(5, 1, 2);
    plt::named_plot("velocity (m/s)", velocity);
    plt::legend();

    plt::subplot(5, 1, 3);
    plt::named_plot("cte (m)", cte);
    plt::named_plot("epsi (rad)", epsi);
    plt::legend();

    plt::subplot(5, 1, 4);
    plt::named_plot("hist. v (m/s)", velocityData_);
    plt::legend();

    plt::subplot(5, 1, 5);
    plt::named_plot("hist. steering (rad)", steeringData_);
    plt::legend();

    plt::pause(0.001);
      
  };  
};