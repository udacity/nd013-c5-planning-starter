/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 **/

#include "behavior_planner_FSM.h"

SharedPtr<Waypoint> BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction) {
  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

  if (_active_maneuver == STOPPED) {
    return waypoint_0;
  }

  // Waypoints at a lookahead distance
  // NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane. The list contains one waypoint for each
  // deviation possible.

  // NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d"
  // apart. The list goes from the current waypoint to the end of its
  // lane.

  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    LOG(INFO) << "Goal wp is a nullptr";
    return nullptr;
  }
  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];
  is_goal_junction = waypoint_0->IsJunction();
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      is_goal_junction = false;
    } else {
      _prev_junction_id = cur_junction_id;
    }
  }

  LOG(INFO) << "is_goal_junction? " << is_goal_junction;
  return waypoint_0;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  auto look_ahead_distance =
      velocity_mag * _lookahead_time +
      0.5 * accel_mag * _lookahead_time * _lookahead_time;

  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);

  LOG(INFO) << "Final look_ahead_distance: " << look_ahead_distance;

  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State& ego_state,
                                   SharedPtr<cc::Map> map) {
  // Get look-ahead distance based on Ego speed
  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  State goal = state_transition(ego_state, goal_wp, is_goal_in_junction);

  return goal;
}

State BehaviorPlannerFSM::state_transition(
    const State& ego_state, const SharedPtr<cc::Waypoint>& goal_wp,
    bool& is_goal_in_junction) {
  // Check with the Behavior Planner to see what are we going to do and
  // where is our next goal
  //
  State goal;

  auto goal_wp_transform = goal_wp->GetTransform();
  goal.location = goal_wp_transform.location;
  goal.rotation = goal_wp_transform.rotation;

  goal.rotation.yaw = utils::deg2rad(goal_wp_transform.rotation.yaw);
  goal.rotation.pitch = utils::deg2rad(goal_wp_transform.rotation.pitch);
  goal.rotation.roll = utils::deg2rad(goal_wp_transform.rotation.roll);

  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  if (_active_maneuver == FOLLOW_LANE) {
    LOG(INFO) << "BP- FOLLOW_LANE";
    if (is_goal_in_junction) {
      LOG(INFO) << "BP - goal in junction";
      goal.velocity.x = 0;
      goal.velocity.y = 0;
      goal.velocity.z = 0;
      _active_maneuver = DECEL_TO_STOP;
    } else {
      goal.velocity.x = 0;  //_speed_limit * std::cos(goal.rotation.yaw);
      goal.velocity.y = 0;  //_speed_limit * std::sin(goal.rotation.yaw);
      goal.velocity.z = 0;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    LOG(INFO) << "BP- DECEL_TO_STOP";
    // Let's backup a "buffer" distance behind the "STOP" point
    auto ang = goal.rotation.yaw + M_PI;
    auto dist = utils::magnitude(goal.location);  // This should be done in 2D
    goal.location.x += dist * std::cos(ang);
    goal.location.y += dist * std::sin(ang);
    // goal.location.z += dist * std::sin(ang); //this must change too wrt to
    // pitch

    goal.velocity.x = 0;
    goal.velocity.y = 0;
    goal.velocity.z = 0;

    auto distance_to_stop_sign =
        utils::magnitude(goal.location - ego_state.location);
    LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;

    if (utils::magnitude(ego_state.velocity) <= _stop_threshold_speed) {
      _active_maneuver = STOPPED;
      _start_stop_time = std::chrono::high_resolution_clock::now();
    }
  } else if (_active_maneuver == STOPPED) {
    LOG(INFO) << "BP- STOPPED";
    goal = ego_state;  // Stay where you are.
    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();
    LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";
    if (stopped_secs >= _req_stop_time) {
      _active_maneuver = FOLLOW_LANE;
    }
  }

  return goal;
}