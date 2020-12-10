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

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
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
  LOG(INFO) << "BP - Num of Lookahead waypoints: " << n_wp;

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0->IsJunction();
  LOG(INFO) << "BP - Is Last wp in juntion? (0/1): " << is_goal_junction;
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      LOG(INFO) << "BP - Last wp is in same juntion as ego. Juntion ID: "
                << _prev_junction_id;
      is_goal_junction = false;
    } else {
      LOG(INFO) << "BP - Last wp is in different juntion than ego. Juntion ID: "
                << cur_junction_id;
      _prev_junction_id = cur_junction_id;
    }
  }
  return waypoint_0;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // TODO-Lookahead: One way to find a reasonable lookahead distance is to find
  // the distance you will need to come to a stop while traveling at speed V and
  // using a comfortable deceleration.

  auto look_ahead_distance = 7000;

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

  LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;

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
    LOG(INFO) << "BP- IN FOLLOW_LANE STATE";
    if (is_goal_in_junction) {
      LOG(INFO) << "BP - goal in junction";

      _active_maneuver = DECEL_TO_STOP;
      LOG(INFO) << "BP - changing to DECEL_TO_STOP";

      // Let's back up a "buffer" distance behind the "STOP" point
      LOG(INFO) << "BP- original STOP goal at: " << goal.location.x << ", "
                << goal.location.y;

      // TODO-goal behind the stopping point: put the goal behind the stopping
      // point (i.e the actual goal location) by "_stop_line_buffer". HINTS:
      // remember that we need to go back in the opposite direction of the
      // goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then
      // use cosine and sine to get x and y
      //
      auto ang = goal.rotation.yaw + M_PI;
      // goal.location.x += ...;
      // goal.location.y += ...;

      LOG(INFO) << "BP- new STOP goal at: " << goal.location.x << ", "
                << goal.location.y;

      // TODO-goal speed at stopping point: What should be the goal speed??
      // goal.velocity.x = ...;
      // goal.velocity.y = ...;
      // goal.velocity.z = ...;

    } else {
      // TODO-goal speed in nominal state: What should be the goal speed now
      // that we know we are in Nominal state and we can continue freely??
      // Remember that the speed is a vector!!

      // goal.velocity.x = ...;
      // goal.velocity.y = ...;
      // goal.velocity.z = ...;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";
    // TODO-maintain the same goal when in DECEL_TO_STOP state: Make sure the
    // new goal is the same as the previous goal (_goal). That way we
    // keep/maintain the goal at the stop line.
    // goal = ...;

    // TODO: It turns out that when we teleport, the car is always at speed
    // zero. In this the case, as soon as we enter the DECEL_TO_STOP state,
    // the condition that we are <= _stop_threshold_speed is ALWAYS true and we
    // move straight to "STOPPED" state. To solve this issue (since we don't
    // have a motion controller yet), you should use "distance" instead of
    // speed. Make sure the distance to the stopping point is <=
    // P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the
    // distance
    auto distance_to_stop_sign = 200;
    // utils::magnitude(goal.location - ego_state.location);
    LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;

    if (utils::magnitude(ego_state.velocity) <= _stop_threshold_speed) {
      // TODO-use distance rather than speed: Use distance rather than speed...
      // if (distance_to_stop_sign <= ...) {

      // TODO-move to STOPPED state: Now that we know we are close or at the
      // stopping point we should change state to "STOPPED"
      //_active_maneuver = ...;

      _start_stop_time = std::chrono::high_resolution_clock::now();
      LOG(INFO) << "BP - changing to STOPPED";
    }
  } else if (_active_maneuver == STOPPED) {
    LOG(INFO) << "BP- IN STOPPED STATE";
    // TODO-maintain the same goal when in STOPPED state: Make sure the new goal
    // is the same as the previous goal. That way we keep/maintain the goal at
    // the stop line. goal = ...;

    // Let's count the time we are stopped
    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();
    LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";

    if (stopped_secs >= _req_stop_time) {
      // TODO-move to FOLLOW_LANE state: What state do we want to move to, when
      // we are "done" at the STOPPED state?
      //_active_maneuver = ...;
      LOG(INFO) << "BP - changing to FOLLOW_LANE";
    }
  }
  _goal = goal;
  return goal;
}