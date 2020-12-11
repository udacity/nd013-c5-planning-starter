/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions {
// COST FUNCTIONS

// double lane_change_cost(const Trajectory trajectory, int dir) {
//   /*
//   Binary cost function which penalizes RIGHT lane changes Vs LEFT lane
//   changes. Gives priority to Left lane changes Vs Right Lane changes (if both
//   abailable).
//   */
//   return (trajectory.maneuver == LANE_CHANGE && dir == RIGHT)
//              ? RIGHT_LANE_CHANGE
//              : 0.0;
// }

double time_diff_cost(const Trajectory trajectory, double T) {
  /*
  Penalizes trajectories that span a duration which is longer or
  shorter than the duration requested (T)f.
  */

  // the last element on the trajectory should hold the last time value, i.e the
  // total duration
  double t = trajectory.duration;
  return TIME_DIFF * (logistic(fabs(t - T) / T));
}

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight) {
  /*
  Penalizes trajectories whose coordinate(and derivatives)
  differ from the goal.
  */
  double cost = 0.0;
  vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
  //////////////cout << "26 - Evaluating f and N derivatives Done. Size:" <<
  /// evals.size() << endl;

  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);
    cost += logistic(diff / sigma[i]);
  }
  ////////////cout << "diff_coeff Cost Calculated " << endl;
  return cost_weight * cost;
}

double collision_and_proximity_cost(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles) {
  /*
  1) Binary cost function which penalizes collisions.
  2) Penalizes getting close to other vehicles.
  */
  // LOG(INFO) << "Collision & Prox. Cost... " << endl;
  double nearest = closest_distance_to_any_vehicle(trajectory, obstacles, dt);
  auto collision_cost = (nearest == 0.0) ? COLLISION : 0.0;
  auto proximity_cost = (logistic(2 * MIN_FOLLOW_DISTANCE / nearest)) * DANGER;
  if (collision_cost > 0.0) {
    LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
  }

  return collision_cost + proximity_cost;
}

// double stays_on_road_cost(const Trajectory trajectory, Road road, double
// EgoWidth)
// {
//   ////////////cout << "stays_on_road_cost" << endl;
//   double RoadWidth = road.num_lanes * road.lane_width;

//   double time_step = dt;  // max(trajectory.duration / 50.0, dt);
//   double t = 0.0;
//   bool OnRoad = true;

//   while (OnRoad && t < trajectory.duration) {
//     double d = evaluate(trajectory.y_coeff, t);
//     OnRoad = ((d > EgoWidth) && (d < (RoadWidth - EgoWidth)));
//     t += time_step;
//   }
//   return DANGER * ((OnRoad) ? 0 : 1);
// }

// double exceeds_speed_limit_cost(const Trajectory trajectory, Road road) {
//   ////////////cout << "exceeds_speed_limit_cost" << endl;
//   double time_step = dt;  // max(trajectory.duration / 50.0, dt);
//   double t = 0.0;
//   vector<double> x_dot = differentiate(trajectory.x_coeff);
//   bool UnderSpeeLimit = true;
//   while (UnderSpeeLimit && t < trajectory.duration) {
//     double Speed = evaluate(x_dot, t);
//     UnderSpeeLimit = (Speed <= road.speed_limit);
//     t += time_step;
//   }
//   return DANGER * ((UnderSpeeLimit) ? 0 : 1);
// }

double efficiency_cost(const Trajectory trajectory, float goal_vx) {
  /*
  Rewards high average Forward speeds. X is considered fwd since we are in Ego
  Coord Frame
  */
  // LOG(INFO) << "efficiency_cost..." << endl;

  double distance_traveled = evaluate(trajectory.x_coeff, trajectory.duration) -
                             evaluate(trajectory.x_coeff, 0.0);
  double avg_v = distance_traveled / trajectory.duration;

  return EFFICIENCY * ((logistic(2 * (goal_vx - avg_v) / avg_v)) + 0.5);
}

double accel_cost_lat_lon(const Trajectory trajectory) {
  // LOG(INFO) << "accel_cost_lat_lon cost..." << endl;
  // Longitudinal
  vector<double> x_dot = differentiate(trajectory.x_coeff);
  vector<double> x_accel = differentiate(x_dot);
  auto longitudinal_accel_cost =
      accel_cost(x_accel, trajectory.duration, CONFORT_MAX_LON_ACCEL,
                 CONFORT_ACCUM_LON_ACC_IN_ONE_SEC);

  // Lateral
  vector<double> y_dot = differentiate(trajectory.y_coeff);
  vector<double> y_accel = differentiate(y_dot);
  auto lateral_accel_cost =
      accel_cost(y_accel, trajectory.duration, CONFORT_MAX_LAT_ACCEL,
                 CONFORT_ACCUM_LAT_ACC_IN_ONE_SEC);

  return longitudinal_accel_cost + lateral_accel_cost;
}

double accel_cost(vector<double> accel, double duration,
                  double CONFORT_MAX_ACCEL,
                  double CONFORT_ACCUM_ACC_IN_ONE_SEC) {
  double time_step = dt;  // max(trajectory.duration / 50.0, dt);
  vector<double> all_accs;
  double total_acc = 0.0;

  for (double t = 0.0; t <= duration; t += time_step) {
    all_accs.push_back(evaluate(accel, t));
    total_acc += fabs(evaluate(accel, t) * time_step);
  }
  double max_acc = *std::max_element(all_accs.begin(), all_accs.end());
  double acc_per_second = total_acc / duration;

  auto max_accel_cost =
      MAX_ACCEL * ((fabs(max_acc) > CONFORT_MAX_ACCEL) ? 1 : 0);
  auto total_accel_cost =
      TOTAL_ACCEL * (logistic(acc_per_second / CONFORT_ACCUM_ACC_IN_ONE_SEC));

  return max_accel_cost + total_accel_cost;
}

double jerk_cost_lat_lon(const Trajectory trajectory) {
  // LOG(INFO) << "jerk_cost_lat_lon cost..." << endl;
  // Longitudinal
  vector<double> x_dot = differentiate(trajectory.x_coeff);
  vector<double> x_d_dot = differentiate(x_dot);
  vector<double> x_jerk = differentiate(x_d_dot);
  auto longitudinal_jerk_cost =
      jerk_cost(x_jerk, trajectory.duration, CONFORT_MAX_LON_JERK,
                CONFORT_ACCUM_LON_JERK_IN_ONE_SEC);

  // Lateral
  vector<double> y_dot = differentiate(trajectory.y_coeff);
  vector<double> y_d_dot = differentiate(y_dot);
  vector<double> y_jerk = differentiate(y_d_dot);
  auto lateral_jerk_cost =
      jerk_cost(y_jerk, trajectory.duration, CONFORT_MAX_LAT_JERK,
                CONFORT_ACCUM_LAT_JERK_IN_ONE_SEC);

  return longitudinal_jerk_cost + lateral_jerk_cost;
}

double jerk_cost(vector<double> jerk, double duration, double CONFORT_MAX_JERK,
                 double CONFORT_ACCUM_JERK_IN_ONE_SEC) {
  double time_step = dt;  // max(trajectory.duration / 50.0, dt);
  vector<double> all_jerks;
  double total_jerk = 0.0;
  for (double t = 0.0; t <= duration; t += time_step) {
    all_jerks.push_back(evaluate(jerk, t));
    total_jerk += fabs(evaluate(jerk, t) * time_step);
  }
  double max_jerk = *std::max_element(all_jerks.begin(), all_jerks.end());
  double jerk_per_second = total_jerk / duration;

  auto max_jerk_cost = MAX_JERK * ((fabs(max_jerk) > CONFORT_MAX_JERK) ? 1 : 0);
  auto total_jerk_cost =
      TOTAL_JERK * (logistic(jerk_per_second / CONFORT_ACCUM_JERK_IN_ONE_SEC));
  return max_jerk_cost + total_jerk_cost;
}

double closest_distance_to_any_vehicle(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles, double time_step) {
  /*
  Calculates the closest distance to any vehicle during a trajectory.
  */
  // LOG(INFO) << "closest_distance_to_any_vehicle..." << std::endl;
  double closest = DBL_MAX;
  for (auto actor : obstacles) {
    double dist = closest_distance_to_vehicle(trajectory, actor, time_step);
    if (dist < closest) {
      closest = dist;
    }
    if (dist == 0.0)  // Return value when there is a collision
    {
      return 0.0;
    }
  }
  return closest;
}

double closest_distance_to_vehicle(const Trajectory trajectory,
                                   const SharedPtr<cc::Actor>& actor,
                                   double time_step) {
  /*
  Calculates the closest distance to a the provided vehicle during a trajectory.
  */
  // LOG(INFO) << "closest_distance_to_vehicle..." << std::endl;
  // LOG(INFO) << "trajectory.duration: " << trajectory.duration << std::endl;

  // For static Obstacles we can just look at the position of the obstacle once,
  // othjerwise we should put it inside the for-loop and calculate the position
  // of the obstacle for evert time step
  auto actor_location = actor->GetLocation();

  double closest = DBL_MAX;
  for (double t = 0.0; t <= trajectory.duration; t += time_step) {
    double cur_x = evaluate(trajectory.x_coeff, t);
    double cur_y = evaluate(trajectory.y_coeff, t);

    double dist = sqrt(pow((cur_x - actor_location.x), 2) +
                       pow((cur_y - actor_location.y), 2));
    if (dist < closest) {
      closest = dist;
    }
    if (dist < 1.5)  // there is a collision for sure..since we are just
                     // comparing point mass!
    {
      return 0.0;
    }
  }
  return closest;
}

double collision_circles_cost_MJT(
    const Trajectory trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles) {
  bool collision{false};
  double min_distance{std::numeric_limits<double>::infinity()};
  auto n_circles = CIRCLE_OFFSETS.size();

  for (double t = dt; t <= trajectory.duration; t += dt) {
    double cur_x = evaluate(trajectory.x_coeff, t);
    double cur_y = evaluate(trajectory.y_coeff, t);
    // Let's calculate the yaw...
    double prev_x = evaluate(trajectory.x_coeff, t - dt);
    double prev_y = evaluate(trajectory.y_coeff, t - dt);
    auto delta_x = cur_x - prev_x;
    auto delta_y = cur_y - prev_y;
    auto cur_yaw = std::atan2(delta_y, delta_x);

    for (size_t c = 0; c < n_circles; c++) {
      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);

      for (auto actor : obstacles) {
        auto actor_location = actor->GetLocation();
        double dist = sqrt(pow((circle_center_x - actor_location.x), 2) +
                           pow((circle_center_y - actor_location.y), 2));
        collision = (dist < CIRCLE_RADII[c]);
        if (collision) {
          break;
        }
        min_distance = std::min(min_distance, dist);
      }

      if (collision) {
        break;
      }
    }
    if (collision) {
      LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
      break;
    }
  }
  double proximity_cost = utils::logistic(1 / min_distance) * DANGER;
  return (collision) ? COLLISION : proximity_cost;
}

double collision_circles_cost_spiral(
    const std::vector<cg::Transform> spiral,
    const std::vector<SharedPtr<cc::Actor>>& obstacles) {
  bool collision{false};
  auto n_circles = CIRCLE_OFFSETS.size();

  for (auto wp : spiral) {
    double cur_x = wp.location.x;
    double cur_y = wp.location.y;
    double cur_yaw = wp.rotation.yaw;  // This is already in rad.

    for (size_t c = 0; c < n_circles; c++) {
      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);

      for (auto actor : obstacles) {
        auto actor_location = actor->GetLocation();
        double dist = sqrt(pow((circle_center_x - actor_location.x), 2) +
                           pow((circle_center_y - actor_location.y), 2));
        collision = (dist < CIRCLE_RADII[c]);
        if (collision) {
          break;
        }
      }

      if (collision) {
        break;
      }
    }
    if (collision) {
      LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
      break;
    }
  }
  return (collision) ? COLLISION : 0.0;
}

double close_to_main_goal_cost_spiral(const std::vector<cg::Transform> spiral,
                                      State main_goal) {
  // The last point on the spiral should be used to check how close we are to
  // the Main (center) goal. That way, goals that are closer to the center lane,
  // that are not in collision, they will be prefered.
  double diff =
      utils::magnitude(spiral[spiral.size() - 1].location - main_goal.location);

  auto cost = logistic(diff);
  LOG(INFO) << "distance to main goal: " << diff;
  LOG(INFO) << "cost (log): " << cost;
  return cost;
}
}  // namespace cost_functions
