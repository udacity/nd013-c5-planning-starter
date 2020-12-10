/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file min_jerk_trajectory_optimizer.h
 **/

#pragma once

#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "cost_functions.h"
#include "structs.h"

namespace cg = carla::geom;
namespace cc = carla::client;
namespace cf = cost_functions;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

class MinJerkTrajectory {
 private:
  State _goal;

 public:
  MinJerkTrajectory();
  ~MinJerkTrajectory();

  Trajectory polynomial_trajectory_generator(const State& ego_state,
                                             const State& goal);

  std::vector<double> MJT(const std::array<double, 3>& start,
                          const std::array<double, 3>& goal, double T);

  double calculate_cost(const Trajectory& trajectory,
                        const std::vector<SharedPtr<cc::Actor>>& obstacles,
                        const State& goal);
};
