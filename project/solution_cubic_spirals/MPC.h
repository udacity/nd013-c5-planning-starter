/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file MPC.h
 **/

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <carla/client/Vehicle.h>

#include <cppad/cppad.hpp>

#include <glog/logging.h>
#include "Eigen/Core"
#include "planning_params.h"
#include "structs.h"
#include "utils.h"

using namespace Eigen;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

// LET'S DEFINE SOME OF THE MPC CONSTANTS

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// WB was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the Wheel Base (WB) of the vehicle, i.e. distance from the center of
// the rear wheel to the center of the front wheel.
const double WB = 2.67;

class MPC {
 public:
  MPC(SharedPtr<carla::client::Vehicle> vehicle);

  virtual ~MPC();
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(
      const std::vector<TrajectoryPoint>& trajectory,
      const std::chrono::high_resolution_clock::time_point& start_time);

 private:
  SharedPtr<carla::client::Vehicle> _vehicle;
  float _max_steer_angle{0.0};  // in rad

  std::vector<TrajectoryPoint> trajectory_into_ego_ref(
      const std::vector<TrajectoryPoint>& trajectory);
};
