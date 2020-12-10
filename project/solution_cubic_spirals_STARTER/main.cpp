/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file main.cpp
 **/

#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include "Eigen/QR"

#include "MPC.h"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "pid.h"
#include "planning_params.h"
#include "plot_utils.h"
#include "pure_pursuit.h"
#include "utils.h"
#include "vehicle_dynamic_model.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace cr = carla::road;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace utils;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;
using namespace Eigen;

#define _USE_MATH_DEFINES

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto& RandomChoice(const RangeT& range, RNG&& generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

static auto ParseArguments(int argc, const char* argv[]) {
  EXPECT_TRUE((argc == 2u) || (argc == 4u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 4u ? ResultType{argv[2u], std::stoi(argv[3u])}
                    : ResultType{"localhost", 2000u};
}

State capture_ego_state(const SharedPtr<cc::Vehicle>& ego_vehicle) {
  State ego_state;
  auto ego_transform =
      boost::static_pointer_cast<cc::Actor>(ego_vehicle)->GetTransform();
  ego_state.location = ego_transform.location;

  ego_state.rotation.yaw = utils::deg2rad(
      utils::keep_angle_range_deg(ego_transform.rotation.yaw, -180, 180));
  ego_state.rotation.pitch = utils::deg2rad(
      utils::keep_angle_range_deg(ego_transform.rotation.pitch, -180, 180));
  ego_state.rotation.roll = utils::deg2rad(
      utils::keep_angle_range_deg(ego_transform.rotation.roll, -180, 180));

  ego_state.velocity = ego_vehicle->GetVelocity();
  ego_state.acceleration = ego_vehicle->GetAcceleration();

  LOG(INFO) << "Ego State - Captured";
  LOG(INFO) << "Ego Loc: x=" << ego_state.location.x
            << " y=" << ego_state.location.y;
  LOG(INFO) << "Ego Rot: yaw=" << ego_state.rotation.yaw;
  LOG(INFO) << "Ego Vel: vx=" << ego_state.velocity.x
            << " vy=" << ego_state.velocity.y;
  LOG(INFO) << "Ego Acc: ax=" << ego_state.acceleration.x
            << " ay=" << ego_state.acceleration.y;
  return ego_state;
}

SharedPtr<cc::Actor> spawn_actor(
    std::mt19937_64& rng, const SharedPtr<cc::BlueprintLibrary>& vehicles,
    cg::Transform& transform, cc::World& world, float offset_x = 0.0,
    float offset_y = 0.0) {
  LOG(INFO) << "Span actor started";
  auto vehicle_blueprint = RandomChoice(*vehicles, rng);
  LOG(INFO) << "Vehicle Blueprint selected randomly";

  if (vehicle_blueprint.ContainsAttribute("color")) {
    auto& attribute = vehicle_blueprint.GetAttribute("color");
    vehicle_blueprint.SetAttribute(
        "color", RandomChoice(attribute.GetRecommendedValues(), rng));
  }
  LOG(INFO) << "Vehicle color chosen";

  if (offset_x != 0.0 || offset_y != 0.0) {
    // auto waypoint_0 = world.GetMap()->GetWaypoint(transform.location);
    // auto lookahead_waypoints = waypoint_0->GetNext(offset_x);
    // waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];
    // transform = waypoint_0->GetTransform();
    transform.location += offset_x * transform.GetForwardVector();
    transform.location.y +=
        offset_y;  // * std::cos(utils::deg2rad(transform.rotation.yaw));
  }

  auto actor = world.SpawnActor(vehicle_blueprint, transform);
  LOG(INFO) << "Actor spawned at x: " << transform.location.x
            << " y: " << transform.location.y << " z: " << transform.location.z
            << " yaw: " << deg2rad(transform.rotation.yaw);

  return actor;
}

void Navigate(int argc, const char* argv[]) {
  try {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(10s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a town.
    auto town_name = "Town03";  // RandomChoice(client.GetAvailableMaps(), rng);
    std::cout << "Loading Udacity's world: " << town_name << std::endl;
    auto world = client.LoadWorld(town_name);

    // Get a random ego_vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();

    auto vehicles = blueprint_library->Filter("vehicle");
    // vehicles = [x for x in vehicles if
    // int(x.get_attribute('number_of_wheels')) == 2]
    LOG(INFO) << "Blueprints filtered just to get vehicles";

    // Find a valid spawn point.
    auto map = world.GetMap();
    // auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    auto transform = map->GetRecommendedSpawnPoints()[0];
    // transform.location += 160.0f * transform.GetForwardVector();
    LOG(INFO) << "Valid Spawn point found";

    // Spawn the EGO ego_vehicle
    auto ego_actor = spawn_actor(rng, vehicles, transform, world);
    LOG(INFO) << "Ego Spawned. Id: " << ego_actor->GetDisplayId();
    auto ego_vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
    auto max_steer_angle = utils::deg2rad(
        ego_vehicle->GetPhysicsControl().wheels[0].max_steer_angle / 2.0);

    // Spawn a few cars blocking the lanes in front of Ego
    std::vector<SharedPtr<cc::Actor>> obstacles;

    auto actor = spawn_actor(rng, vehicles, transform, world, 20.0f, -1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 20.0f, 4 * 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 40.0f, 4 * -1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 10.0f, 0.0f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 10.0f, 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 10.0f, 0.0f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 40.0f, 0.0f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 20.0f, 3 * 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 10.0f, 0.0f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 17.0f, -1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    // Move spectator so we can see the ego_vehicle from the simulator window.
    auto spectator = world.GetSpectator();

    // For plotting purposes
    std::vector<State> main_goals;

    // Decalre and initialize the Behavior Planner and all its class
    // requirements
    BehaviorPlannerFSM behavior_planner(
        P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
        P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
        P_MAX_ACCEL, P_STOP_LINE_BUFFER);

    // Decalre and initialized the Motion Planner and all its class requirements
    MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

    // MPC is initialized here!
    // MPC mpc(ego_vehicle);

    // Throttle feedforward + Feedback Controller (PID + Feedforward) -
    // (Longitudinal Control)
    PID throttle_brake_pid(1.90, 0.05, 0.80);
    throttle_brake_pid.set_sample_time(0.033);
    VehicleDynamicModel vehicle_dynamic_model;
    vehicle_dynamic_model.load_vehicle_data(ego_vehicle);

    PID steering_pid(0.34611, 0.0370736, 3.5349);
    steering_pid.set_sample_time(0.033);

    // Pure Pursuit for Steering Control (Lateral Control)
    PurePursuit pp(4.5, 1.00, 1.3);

    while (true) {
      auto start_time = std::chrono::high_resolution_clock::now();

      // Get Ego Current State (Location + Rotation), Velocity and Accel vectors
      auto ego_state = capture_ego_state(ego_vehicle);

      if (ego_state.location.x < transform.location.x) {
        std::cout << " Route finished." << std::endl;
        break;
      }

      // ********** BEHAVIOR PLANNING: FIND A GOAL **************
      LOG(INFO) << "BEHAVIOR PLANNING";
      State goal = behavior_planner.get_goal(ego_state, map);

      main_goals.emplace_back(goal);
      // utils::plot_goals(main_goals);

      LOG(INFO) << "Goal at:";
      LOG(INFO) << "x: " << goal.location.x << " y: " << goal.location.y
                << " z: " << goal.location.z << " yaw: " << goal.rotation.yaw;

      auto distance = utils::magnitude(goal.location - ego_state.location);
      LOG(INFO) << "distance to main goal: " << distance;

      if (behavior_planner.get_active_maneuver() == STOPPED) {
        std::this_thread::sleep_for(1s);
        continue;
      }

      // *************** PLAN MOTION TO GOAL **************
      LOG(INFO) << "MOTION PLANNING";
      LOG(INFO) << " Generating Offset Goals";
      auto goal_set = motion_planner.generate_offset_goals(goal);
      // utils::plot_goals(goal_set);
      LOG(INFO) << goal_set.size() << " Goals Generated";

      LOG(INFO) << " Generating spirals";
      // State ego_state_from_ego_ref;
      auto spirals = motion_planner.generate_spirals(ego_state, goal_set);
      LOG(INFO) << spirals.size() << " Spirals Generated";
      // utils::plot_spirals(spirals);

      // LOG(INFO) << " Spirals back to global frame..";
      // spirals =
      //     motion_planner.transform_spirals_to_global_frame(spirals,
      //     ego_state);
      // LOG(INFO) << spirals.size() << " Spirals back to global frame DONE";
      // // utils::plot_spirals(spirals);

      LOG(INFO) << " Getting Best SPIRAL";
      auto best_spiral_idx =
          motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
      LOG(INFO) << "Best spiral idx: " << best_spiral_idx;

      auto best_spiral = motion_planner._best_spiral;
      if (best_spiral_idx > -1) {
        motion_planner._best_spiral = spirals[best_spiral_idx];
        motion_planner._prev_step_count = 0;
        LOG(INFO) << "Best spiral updated";
      }
      LOG(INFO) << "Best spiral size: " << motion_planner._best_spiral.size();

      ++motion_planner._prev_step_count;
      LOG(INFO) << "_prev_step_count: " << motion_planner._prev_step_count;
      if (motion_planner._prev_step_count >
          motion_planner._best_spiral.size()) {
        std::cout << "We have not found a feasible path and we run out of "
                     "points in the current path"
                  << std::endl;
        break;
      }

      LOG(INFO) << "Generating Velocity Profile";
      // Compute the velocity profile for the path, and compute the waypoints.
      // Use the lead vehicle to inform the velocity profile's dynamic obstacle
      // handling. In this scenario, the only dynamic obstacle is the lead
      // vehicle at index 1.
      auto desired_speed = utils::magnitude(goal.velocity);

      State lead_car_state;  // = to the vehicle ahead...
      auto trajectory =
          motion_planner._velocity_profile_generator.generate_trajectory(
              motion_planner._best_spiral, desired_speed, ego_state,
              lead_car_state, behavior_planner.get_active_maneuver());
      LOG(INFO) << "Velocity Profiler generated a trajectory with "
                << trajectory.size() << " points";

      // // *************** EXECUTE THE MOTION: MOTION CONTROL **************
      // LOG(INFO) << "MOTION CONTROL";

      // // compute the optimal control commands
      // // auto mpc_solution = mpc.Solve(trajectory, start_time);

      // // // (between -1 and +1)
      // // auto steer_cmd = mpc_solution[0];

      // // // (between -1 and +1)
      // // double throttle_cmd = mpc_solution[1];

      // // ####### Feedfwd + Feedback ########
      // auto ego_speed = utils::magnitude(ego_state.velocity);
      // auto speed_err = desired_speed - ego_speed;
      // std::array<double, 2> output_limits = {-1.0, 1.00};
      // throttle_brake_pid.update(speed_err, output_limits);

      // auto throttle_cmd = throttle_brake_pid.output;
      // throttle_cmd += vehicle_dynamic_model.get_throttle(
      //     desired_speed, ego_state.rotation.pitch);
      // double brake_cmd{throttle_cmd};
      // if (throttle_cmd <= 0) {
      //   throttle_cmd = 0.0;
      //   brake_cmd *= -1;
      // } else {
      //   brake_cmd = 0.0;
      // }

      // // // #### PURE PURSUIT & STANLEY ####

      // // // fit a 3rd order polynomial to the waypoints
      // MatrixXd wp_vehRef = pp.transMap2CarRef(ego_state, trajectory);
      // VectorXd wp_vehRef_x = wp_vehRef.row(0);
      // VectorXd wp_vehRef_y = wp_vehRef.row(1);

      // auto coeffs = pp.polyfit(wp_vehRef_x, wp_vehRef_y, 3);
      // // auto steer_angle = pp.update(coeffs, ego_state.rotation.yaw,
      // // ego_speed);
      // // steer_angle = std::clamp(steer_angle, -max_steer_angle,
      // // max_steer_angle); auto steer_cmd = steer_angle / 2 *
      // max_steer_angle; auto cte = pp.polyeval(coeffs, 0.0);
      // steering_pid.update(cte, output_limits);

      // auto steer_cmd = steering_pid.output;

      // LOG(INFO) << "MPC - CMDs ";
      // LOG(INFO) << "St: " << steer_cmd;
      // LOG(INFO) << "Th: " << throttle_cmd;
      // LOG(INFO) << "Bk: " << brake_cmd;

      // // Apply control to ego_vehicle->
      // cc::Vehicle::Control vehicle_control_cmds;
      // vehicle_control_cmds.throttle = throttle_cmd;
      // vehicle_control_cmds.steer = steer_cmd;
      // vehicle_control_cmds.brake = brake_cmd;
      // ego_vehicle->ApplyControl(vehicle_control_cmds);

      // // Move spectator so we can see the ego_vehicle from the simulator
      // // window.
      // auto spectator_transform = ego_vehicle->GetTransform();
      // spectator_transform.location -=
      //     10.0f * spectator_transform.GetForwardVector();
      // spectator_transform.location.z += 5.0f;
      // spectator_transform.rotation.pitch = -15.0f;
      // spectator->SetTransform(spectator_transform);

      // // Give some time to CARLA to move and update all the world
      // std::this_thread::sleep_for(0.25s);

      // // // Display the MPC predicted trajectory
      // // // DisplayMpcPredictedtrajectory(mpc_solution, waypoints_wrt_ego);

      // *************** EXECUTE THE MOTION: TELEPORT **************
      // ONLY FOR TELEPORTATION
      LOG(INFO) << "TELEPORT";
      ego_vehicle->SetSimulatePhysics(false);
      LOG(INFO) << "Ego State at the begginig of TELEPORTATION "
                << " x: " << ego_state.location.x
                << " y: " << ego_state.location.y
                << " z: " << ego_state.location.z
                << " yaw: " << (ego_state.rotation.yaw);

      long long latency_micro_secs =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now() - start_time)
              .count();
      double latency_secs = latency_micro_secs / 1e6;
      LOG(INFO) << "Latency (microsec): " << latency_micro_secs;
      LOG(INFO) << "Latency (sec): " << latency_secs;

      auto n =
          1;  // P_NUM_POINTS_IN_SPIRAL;  // (size_t)(trajectory.size() * 0.05);
      LOG(INFO) << "Teleporting " << n
                << " points from the trajectory starting at "
                << motion_planner._prev_step_count;
      for (size_t i = 0;
           i < n && motion_planner._prev_step_count < trajectory.size(); ++i) {
        auto ego_new_path_point =
            trajectory[motion_planner._prev_step_count].path_point;
        cg::Transform ego_new_transform;
        ego_new_transform.location.x = ego_new_path_point.x;
        ego_new_transform.location.y = ego_new_path_point.y;
        ego_new_transform.location.z = ego_new_path_point.z;
        ego_new_transform.rotation.yaw =
            utils::rad2deg(ego_new_path_point.theta);

        LOG(INFO) << "MOVING TO x: " << ego_new_transform.location.x
                  << " y: " << ego_new_transform.location.y
                  << " z: " << ego_new_transform.location.z
                  << " yaw: " << ego_new_transform.rotation.yaw
                  << " pitch: " << ego_new_transform.rotation.pitch;

        // Teleport ego car to this position and rotation
        ego_vehicle->SetTransform(ego_new_transform);

        // Move spectator so we can see the ego_vehicle from the simulator
        // window.
        auto spectator_transform = ego_new_transform;
        spectator_transform.location -=
            10.0f * spectator_transform.GetForwardVector();
        spectator_transform.location.z += 5.0f;
        spectator_transform.rotation.pitch = -15.0f;
        spectator->SetTransform(spectator_transform);

        // Give some time to CARLA to move and update all the world
        // std::this_thread::sleep_for(0.3s);

        // Let's keep track on what step of the trajectory we are.
        ++motion_planner._prev_step_count;
      }
    };

    // Remove actors from the simulation.
    ego_vehicle->Destroy();
    for (auto actor : obstacles) {
      actor->Destroy();
    }

    std::cout << "Actors destroyed." << std::endl;
  }

  catch (const cc::TimeoutException& e) {
    std::cout << '\n' << e.what() << std::endl;
    return;
  }

  catch (const std::exception& e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return;
  }
}

// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  std::string argv_str(argv[0]);
  std::string exe_folder = argv_str.substr(0, argv_str.find_last_of("/"));
  std::cout << " Logging in: " << exe_folder << std::endl;
  FLAGS_log_dir = exe_folder;
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // const char* logfileName = exe_folder + "/logfile.log";
  google::SetLogDestination(
      google::GLOG_INFO,
      "/home/munir/carla/nd013-c5-planning-refresh/project/"
      "solution_cubic_spirals/bin/logs.log");

  try {
    Navigate(argc, argv);
    std::cout << "Automatic Navigation is done!" << std::endl;
    google::ShutdownGoogleLogging();
    return 0;
  }

  catch (const std::exception& e) {
    std::cout << "\nException: " << e.what() << std::endl;
    google::ShutdownGoogleLogging();
    return 2;
  }
}