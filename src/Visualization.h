#pragma once

#include "constants.h"
#include "matplotlibcpp.h"

#include <cppad/cppad.hpp>
#include <chrono>
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
          plot(data_);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  ~Visualizer() {
    run_ = false;
    worker_.join();
  }

  void setData(Dvector const & solution) {
    data_ = solution;
    dirty_ = true;
  }

private:
  std::thread worker_;
  Dvector data_;
  bool dirty_ {false};
  bool run_{false};


  void plot(Dvector const & solution){
    vector<float> steering;
    vector<float> throttle;
    vector<float> velocity;
    vector<float> psi;
    vector<float> cte;
    vector<float> epsi;

    // fill actuation data
    for(size_t i = 0; i < N-1; ++i) {
      steering.push_back(solution[delta_start+i]);
      throttle.push_back(solution[a_start+i]);
    }

    // fill trajectory data
    for(size_t i = 0; i < N; ++i) {
      velocity.push_back(solution[v_start+i]);
      psi.push_back(solution[psi_start+i]);
      cte.push_back(solution[cte_start+i]);
      epsi.push_back(solution[epsi_start+i]);
    }


    //
    // actual plotting

    // clear previous plot
    plt::clf();
    
    plt::subplot(3, 1, 1);
    plt::named_plot("steering (rad)", steering);
    plt::named_plot("throttle", throttle);
    plt::title("MPC Trajectory");
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::named_plot("velocity (m/s)", velocity);
    plt::named_plot("orientation (rad)", psi);
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::named_plot("cte (m)", cte);
    plt::named_plot("epsi (rad)", epsi);
    plt::legend();

    plt::pause(0.001);
      
  };
protected:
  
};