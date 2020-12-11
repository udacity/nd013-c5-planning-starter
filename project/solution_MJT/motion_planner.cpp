/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file motion_planner.cpp
 **/

#include "motion_planner.h"

MotionPlanner::~MotionPlanner() {}

State MotionPlanner::get_goal_state_in_ego_frame(const State& ego_state,
                                                 const State& goal_state) {
  // Let's start by making a copy of the goal state (global reference frame)
  auto goal_state_ego_frame = goal_state;

  // Translate so the ego state is at the origin in the new frame.
  // This is done by subtracting the ego_state from the goal_ego_.
  goal_state_ego_frame.location.x -= ego_state.location.x;
  goal_state_ego_frame.location.y -= ego_state.location.y;
  goal_state_ego_frame.location.z -= ego_state.location.z;

  /* Rotate such that the ego state has zero heading/yaw in the new frame.
     We are rotating by -ego_state "yaw" to ensure the ego vehicle's
     current yaw corresponds to theta = 0 in the new local frame.

     Recall that the general rotation matrix around the Z axix is:
     [cos(theta) -sin(theta)
     sin(theta)  cos(theta)]
  */
  auto theta_rad = -ego_state.rotation.yaw;
  auto cos_theta = std::cos(theta_rad);
  auto sin_theta = std::sin(theta_rad);

  goal_state_ego_frame.location.x =
      cos_theta * goal_state_ego_frame.location.x -
      sin_theta * goal_state_ego_frame.location.y;
  goal_state_ego_frame.location.y =
      sin_theta * goal_state_ego_frame.location.x +
      cos_theta * goal_state_ego_frame.location.y;

  // Compute the goal yaw in the local frame by subtracting off the
  // current ego yaw from the goal waypoint heading/yaw.
  goal_state_ego_frame.rotation.yaw += theta_rad;

  // Ego speed is the same in both coordenates
  // the Z coordinate does not get affected by the rotation.

  // Let's make sure the yaw is within [-180, 180] or [-pi, pi] so the optimizer
  // works.
  goal_state_ego_frame.rotation.yaw = utils::keep_angle_range_rad(
      goal_state_ego_frame.rotation.yaw, -M_PI, M_PI);

  return goal_state_ego_frame;
}

std::vector<State> MotionPlanner::generate_offset_goals_ego_frame(
    const State& ego_state, const State& goal_state) {
  // Let's transform the "main" goal (goal state) into ego reference frame
  auto goal_state_ego_frame =
      get_goal_state_in_ego_frame(ego_state, goal_state);

  return generate_offset_goals(goal_state_ego_frame);
}

std::vector<State> MotionPlanner::generate_offset_goals_global_frame(
    const State& goal_state) {
  return generate_offset_goals(goal_state);
}

std::vector<State> MotionPlanner::generate_offset_goals(
    const State& goal_state) {
  // Now we need to gernerate "_num_paths" goals offset from the center goal at
  // a distance "_goal_offset".
  std::vector<State> goals_offset;

  // the goals will be aligned on a perpendiclular line to the heading of the
  // main goal. To get a perpendicular angle, just add 90 (or PI/2) to the main
  // goal heading.
  auto yaw = goal_state.rotation.yaw + M_PI_2;

  LOG(INFO) << "MAIN GOAL";
  LOG(INFO) << "x: " << goal_state.location.x << " y: " << goal_state.location.y
            << " z: " << goal_state.location.z
            << " yaw (rad): " << goal_state.rotation.yaw;
  LOG(INFO) << "OFFSET GOALS";
  LOG(INFO) << "ALL offset yaw (rad): " << yaw;

  for (int i = 0; i < _num_paths; ++i) {
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset;
    LOG(INFO) << "Goal: " << i + 1;
    LOG(INFO) << "(int)(_num_paths / 2): " << (int)(_num_paths / 2);
    LOG(INFO) << "(i - (int)(_num_paths / 2)): " << (i - (int)(_num_paths / 2));
    LOG(INFO) << "_goal_offset: " << _goal_offset;

    LOG(INFO) << "offset: " << offset;
    goal_offset.location.x += offset * std::cos(yaw);
    goal_offset.location.y += offset * std::sin(yaw);
    LOG(INFO) << "x: " << goal_offset.location.x
              << " y: " << goal_offset.location.y
              << " z: " << goal_offset.location.z
              << " yaw (rad): " << goal_offset.rotation.yaw;

    if (valid_goal(goal_state, goal_offset)) {
      goals_offset.push_back(goal_offset);
    }
  }
  return goals_offset;
}

bool MotionPlanner::valid_goal(State main_goal, State offset_goal) {
  auto max_offset = ((int)(_num_paths / 2) + 1) * _goal_offset;
  LOG(INFO) << "max offset: " << max_offset;
  auto dist = utils::magnitude(main_goal.location - offset_goal.location);
  LOG(INFO) << "distance from main goal to offset goal: " << dist;
  return dist < max_offset;
}

std::vector<Trajectory> MotionPlanner::generate_min_jerk_trajectories(
    const State& ego_state, const std::vector<State>& goals) {
  // Create the return vector.
  std::vector<Trajectory> trajectories;

  for (auto goal : goals) {
    auto trajectory =
        _MJT_optimizer.polynomial_trajectory_generator(ego_state, goal);
    if (valid_trajectory(trajectory, goal)) {
      LOG(INFO) << "Trajectory Valid ";
      trajectories.push_back(trajectory);
    } else {
      LOG(INFO) << "Trajectory Invalid ";
    }
  }
  return trajectories;
}

int MotionPlanner::get_best_trajectory_idx(
    const std::vector<Trajectory>& trajectories,
    const std::vector<SharedPtr<cc::Actor>>& obstacles,
    const State& goal_state) {
  LOG(INFO) << "Best Trajectory Idx...";
  LOG(INFO) << "trajectories.size(): " << trajectories.size();
  double best_cost = DBL_MAX;
  int best_trajectory_idx = -1;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    double cost =
        _MJT_optimizer.calculate_cost(trajectories[i], obstacles, goal_state);
    if (cost < best_cost) {
      best_cost = cost;
      best_trajectory_idx = i;
    }
  }
  return best_trajectory_idx;
}

bool MotionPlanner::valid_trajectory(const Trajectory& trajectory,
                                     const State& offset_goal) {
  cg::Location traj_end;
  traj_end.x = utils::evaluate(trajectory.x_coeff, trajectory.duration);
  traj_end.y = utils::evaluate(trajectory.y_coeff, trajectory.duration);
  traj_end.z = utils::evaluate(trajectory.z_coeff, trajectory.duration);

  auto dist = utils::magnitude(traj_end - offset_goal.location);
  LOG(INFO) << "Distance from Trajectory end to offset_goal: " << dist;
  return (dist < 0.5);
}