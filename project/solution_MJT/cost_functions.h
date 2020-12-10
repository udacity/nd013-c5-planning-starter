/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.h
 **/

#pragma once

#include <math.h>

#include <algorithm>  // std::max, min
#include <cfloat>
#include <iostream>
#include <limits>
#include <numeric>  // std::accumulate
#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "planning_params.h"
#include "structs.h"
#include "utils.h"

using namespace std;
using namespace utils;
namespace cc = carla::client;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

// priority levels for costs: Higher COST will give you a higher priority to
// AVOID

const double TIME_DIFF = 1e1;
const double X_DIFF = 1e12;
const double Y_DIFF = 1e12;
const double EFFICIENCY = 1e3;
const double MAX_JERK = 1e8;
const double TOTAL_JERK = 1e7;
const double COLLISION = std::numeric_limits<double>::infinity();
const double DANGER = 1e3;
const double MAX_ACCEL = 1e8;
const double TOTAL_ACCEL = 1e8;
const double RIGHT_LANE_CHANGE = 1e1;

// Average Vehicle {Length, Width} in meters
// (https://en.wikipedia.org/wiki/Family_car)
const vector<double> VEHICLE_SIZE = {5, 2};

// is the minimum distance in case of congestion (v = 0).
const double MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0];

namespace cost_functions {
// COST FUNCTIONS

double time_diff_cost(const Trajectory trajectory, double T);

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight);

double collision_and_proximity_cost(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles);

double collision_circles_cost_MJT(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles);

double collision_circles_cost_spiral(
    const std::vector<cg::Transform> spiral,
    const std::vector<SharedPtr<cc::Actor>>& obstacles);

double close_to_main_goal_cost_spiral(const std::vector<cg::Transform> spiral,
                                      State goal);

// double stays_on_road_cost(const Trajectory trajectory, Road road, double
// EgoWidth);

// double exceeds_speed_limit_cost(const Trajectory trajectory, Road road);

double efficiency_cost(const Trajectory trajectory, float goal_vx);

double accel_cost_lat_lon(const Trajectory trajectory);

double accel_cost(vector<double> accel, double duration,
                  double CONFORT_MAX_ACCEL,
                  double CONFORT_ACCUM_ACC_IN_ONE_SEC);

double jerk_cost_lat_lon(const Trajectory trajectory);

double jerk_cost(vector<double> jerk, double duration, double CONFORT_MAX_JERK,
                 double CONFORT_ACCUM_JERK_IN_ONE_SEC);

double closest_distance_to_any_vehicle(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles, double time_step);

double closest_distance_to_vehicle(const Trajectory trajectory,
                                   const SharedPtr<cc::Actor>& actor,
                                   double time_step);
}  // namespace cost_functions