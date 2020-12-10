/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file motion_planner.h
 **/

#pragma once

#include <math.h>

#include <cfloat>
#include <memory>
#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "cost_functions.h"
#include "min_jerk_trajectory_optimizer.h"
#include "planning_params.h"
#include "structs.h"
#include "utils.h"

namespace cc = carla::client;
namespace cost = cost_functions;
namespace cg = carla::geom;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;

class MotionPlanner {
 private:
  unsigned short _num_paths;  // number of lateral offset paths to generate.
  float _goal_offset;         // lateral distance between goals.
  float _error_tolerance;
  MinJerkTrajectory _MJT_optimizer;

 public:
  MotionPlanner(unsigned short num_paths, float goal_offset,
                float error_tolerance)
      : _num_paths(num_paths),
        _goal_offset(goal_offset),
        _error_tolerance(error_tolerance) {}

  ~MotionPlanner();

  Trajectory _best_trajectory;
  std::vector<cg::Transform> _best_spiral;
  size_t _prev_step_count{0};

  State get_goal_state_in_ego_frame(const State& ego_state,
                                    const State& goal_state);

  std::vector<State> generate_offset_goals_ego_frame(const State& ego_state,
                                                     const State& goal_state);
  std::vector<State> generate_offset_goals_global_frame(
      const State& goal_state);
  std::vector<State> generate_offset_goals(const State& goal_state);

  std::vector<Trajectory> generate_min_jerk_trajectories(
      const State& ego_state, const std::vector<State>& goals);

  int get_best_trajectory_idx(
      const std::vector<Trajectory>& trajectories,
      const std::vector<SharedPtr<cc::Actor>>& obstacles,
      const State& goal_state);

  bool valid_goal(State main_goal, State offset_goal);
  bool valid_trajectory(const Trajectory& trajectory, const State& offset_goal);
};