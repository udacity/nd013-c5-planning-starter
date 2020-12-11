/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file min_jerk_trajectory_optimizer.cpp
 **/

#include "min_jerk_trajectory_optimizer.h"

#include <algorithm>

#include <carla/client/Client.h>
#include "Eigen/Core"
#include "Eigen/LU"

#include "utils.h"

using namespace Eigen;

MinJerkTrajectory::MinJerkTrajectory(/* args */) {}

MinJerkTrajectory::~MinJerkTrajectory() {}

std::vector<double> MinJerkTrajectory::MJT(const std::array<double, 3>& start,
                                           const std::array<double, 3>& goal,
                                           double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
  corresponding to initial values of [x, x_dot, x_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
  length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the
  polynomial s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 *
  t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  double a0 = start[0];
  double a1 = start[1];
  double a2 = 0.5 * start[2];

  // Let's solve the rest assuming a 3 equations with 3 unkowns in the form of
  // Ax = B
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;
  Eigen::MatrixXd A(3, 3);
  // MatrixX3d A;

  A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

  VectorXd B(3);
  B << goal[0] - (a0 + a1 * T + a2 * T2), goal[1] - (a1 + 2 * a2 * T),
      goal[2] - 2 * a2;

  // If T = 0, then A's determinant is 0 and is NOT invertible and we can
  // confortably pass back same status + 0, 0, 0..
  VectorXd C(3);
  if (!T == 0)
    C = A.inverse() * B;
  else
    C << 0, 0, 0;

  return {a0, a1, a2, C[0], C[1], C[2]};
}

Trajectory MinJerkTrajectory::polynomial_trajectory_generator(
    const State& ego_state, const State& goal) {
  /*
  POLYNOMIAL TRAJECTORY GENERATION

  Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS(global).

  arguments :
  start_x - [x, x_dot, x_ddot]

  start_y - [y, y_dot, y_ddot]

  goal_x - [x, x_dot, x_ddot]

  goal_y - [y, y_dot, y_ddot]

  T - the desired time at which we will be at the goal(relative to now as t = 0)

  return:
  vector of vectors:(best_s, best_d, best_t, goal_state.location.x,
  goal_state.location.y) where: best_s are the 6 coefficients representing s(t)
  best_d gives coefficients for d(t) and best_t gives duration associated w /
  this trajectory.
  */

  // Let's estimate the Time duration of the manouver.
  // CRAZY: Assume rectilinear motion and add a little bit more time!!??

  auto ego_speed = utils::magnitude(ego_state.velocity);
  auto distance = utils::magnitude(goal.location - ego_state.location);

  auto accel = CONFORT_MAX_LON_ACCEL;
  std::array<double, 2> traj_duration_sol =
      utils::solve_quadratic((accel / 2), ego_speed, -distance);

  LOG(INFO) << "Traj Duration for a: " << (accel / 2) << " b: " << ego_speed
            << " c: " << -distance << " Sol1: " << traj_duration_sol[0]
            << " Sol2:" << traj_duration_sol[1];
  double T = std::max(traj_duration_sol[0], traj_duration_sol[1]) + 0.5;
  T = std::min(std::max(T, MIN_MANEUVER_TIME), MAX_MANEUVER_TIME);
  LOG(INFO) << "Traj Duration: " << T;
  LOG(INFO) << "Traj Distance: " << distance;

  LOG(INFO) << "Traj Distance: " << distance;

  std::array<double, 3> start_x{ego_state.location.x, ego_state.velocity.x,
                                ego_state.acceleration.x};
  std::array<double, 3> start_y{ego_state.location.y, ego_state.velocity.y,
                                ego_state.acceleration.y};
  std::array<double, 3> start_z{ego_state.location.z, ego_state.velocity.z,
                                ego_state.acceleration.z};

  std::array<double, 3> goal_x{goal.location.x, goal.velocity.x,
                               goal.acceleration.x};
  std::array<double, 3> goal_y{goal.location.y, goal.velocity.y,
                               goal.acceleration.y};
  std::array<double, 3> goal_z{goal.location.z, goal.velocity.z,
                               goal.acceleration.z};

  std::vector<double> x_coeff = MJT(start_x, goal_x, T);
  std::vector<double> y_coeff = MJT(start_y, goal_y, T);
  std::vector<double> z_coeff = MJT(start_z, goal_z, T);

  // CREATE THE TRAJECTORY STRUCTURE
  Trajectory OneTrajectory;
  // OneTrajectory.maneuver = maneuver;
  std::copy(x_coeff.begin(), x_coeff.end(),
            std::back_inserter(OneTrajectory.x_coeff));

  std::copy(y_coeff.begin(), y_coeff.end(),
            std::back_inserter(OneTrajectory.y_coeff));

  std::copy(z_coeff.begin(), z_coeff.end(),
            std::back_inserter(OneTrajectory.z_coeff));

  std::copy(start_x.begin(), start_x.end(),
            std::back_inserter(OneTrajectory.start_x));

  std::copy(start_y.begin(), start_y.end(),
            std::back_inserter(OneTrajectory.start_y));

  std::copy(start_z.begin(), start_z.end(),
            std::back_inserter(OneTrajectory.start_z));

  std::copy(goal_x.begin(), goal_x.end(),
            std::back_inserter(OneTrajectory.goal_x));

  std::copy(goal_y.begin(), goal_y.end(),
            std::back_inserter(OneTrajectory.goal_y));

  std::copy(goal_z.begin(), goal_z.end(),
            std::back_inserter(OneTrajectory.goal_z));

  OneTrajectory.duration = T;

  return OneTrajectory;
}

double MinJerkTrajectory::calculate_cost(
    const Trajectory& trajectory,
    const std::vector<SharedPtr<cc::Actor>>& obstacles, const State& goal) {
  // Initialize cost to 0.0
  double cost = 0.0;

  // bool lane_change = (trajectory[1][0]) - get_lane(trajectory.start_d[0]);
  // cost += lane_change_cost(trajectory, dir);

  // cost += time_diff_cost(trajectory, goal_duration);

  std::array<double, 3> goal_x{goal.location.x, goal.velocity.x,
                               goal.acceleration.x};
  std::array<double, 3> goal_y{goal.location.y, goal.velocity.y,
                               goal.acceleration.y};

  // std::array<double, 3> goal_yaw{goal.rotation.yaw, 0, 0};

  cost += cf::diff_cost(trajectory.x_coeff, trajectory.duration, goal_x,
                        SIGMA_X, EFFICIENCY);

  cost += cf::diff_cost(trajectory.y_coeff, trajectory.duration, goal_y,
                        SIGMA_Y, EFFICIENCY);

  // cost += cf::diff_cost(trajectory.yaw_coeff, trajectory.duration, goal_yaw,
  //                       SIGMA_YAW, EFFICIENCY);

  // cost += cf::collision_and_proximity_cost(trajectory, obstacles);

  cost += cf::collision_circles_cost_MJT(trajectory, obstacles);

  // cost += stays_on_road_cost(trajectory, this->road, VEHICLE_SIZE[0]);

  // cost += exceeds_speed_limit_cost(trajectory, this->road);

  // cost += cf::efficiency_cost(trajectory, goal.velocity.x);

  // cost += cf::accel_cost_lat_lon(trajectory);

  // cost += cf::jerk_cost_lat_lon(trajectory);

  LOG(INFO) << "Traj Cost: " << cost;
  return cost;
}
