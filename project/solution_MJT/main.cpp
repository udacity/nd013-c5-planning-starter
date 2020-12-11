/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file main.cpp
 **/

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

#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "plot_utils.h"
#include "utils.h"

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

  ego_state.rotation.yaw = utils::deg2rad(ego_transform.rotation.yaw);
  ego_state.rotation.pitch = utils::deg2rad(ego_transform.rotation.pitch);
  ego_state.rotation.roll = utils::deg2rad(ego_transform.rotation.roll);

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
    transform.location += offset_x * transform.GetForwardVector();
    transform.location.y += offset_y;
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

    // Spawn a few cars blocking the lanes in front of Ego
    std::vector<SharedPtr<cc::Actor>> obstacles;

    auto actor = spawn_actor(rng, vehicles, transform, world, 10.0f, 4.0f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 20.0f, -5.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 60.0f, 4 * 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 40.0f, -4 * 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    actor = spawn_actor(rng, vehicles, transform, world, 40.0f, 4 * 1.5f);
    // boost::static_pointer_cast<cc::Vehicle>(actor)->SetAutopilot();
    LOG(INFO) << "Actor Spawned. Id: " << actor->GetDisplayId();
    obstacles.push_back(actor);

    // Move spectator so we can see the ego_vehicle from the simulator window.
    auto spectator = world.GetSpectator();

    // For plotting purposes
    std::vector<State> main_goals;

    // Decalre and initialized the Motion Planner and all its class requirements
    MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

    // Decalre and initialize the Behavior Planner and all its class
    // requirements
    BehaviorPlannerFSM behavior_planner(
        P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
        P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
        P_MAX_ACCEL, P_STOP_LINE_BUFFER);

    // ONLY FOR TELEPORTATION
    ego_vehicle->SetSimulatePhysics(false);

    while (true) {
      auto start_time = std::chrono::high_resolution_clock::now();

      // Get Ego Current State (Location + Rotation), Velocity and Accel vectors
      auto ego_state = capture_ego_state(ego_vehicle);

      // ********** BEHAVIOR PLANNING: FIND A GOAL **************
      State goal = behavior_planner.get_goal(ego_state, map);

      main_goals.emplace_back(goal);
      utils::plot_goals(main_goals);

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
      LOG(INFO) << " Generating Goals";
      auto goal_set = motion_planner.generate_offset_goals(goal);
      // utils::plot_goals(goal_set);
      LOG(INFO) << goal_set.size() << " Goals Generated";

      LOG(INFO) << " Generating Trajectories";
      auto trajectories =
          motion_planner.generate_min_jerk_trajectories(ego_state, goal_set);
      LOG(INFO) << "Trajectories Generated";

      LOG(INFO) << " Getting Best Trajectory";
      auto best_trajectory_idx =
          motion_planner.get_best_trajectory_idx(trajectories, obstacles, goal);
      LOG(INFO) << "Best trajectory idx: " << best_trajectory_idx;

      Trajectory best_trajectory = motion_planner._best_trajectory;
      if (best_trajectory_idx > -1) {
        best_trajectory = trajectories[best_trajectory_idx];
        motion_planner._best_trajectory = best_trajectory;
        motion_planner._prev_step_count = 0;
        LOG(INFO) << "Best Traj updated";
      }

      ++motion_planner._prev_step_count;
      LOG(INFO) << "_prev_step_count: " << motion_planner._prev_step_count;

      // *************** EXECUTE THE MOTION: TELEPORT **************
      LOG(INFO) << "Ego State at the begginig of MOTION "
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

      cg::Transform ego_prev_state;
      cg::Transform ego_curr_state;
      cg::Transform ego_next_state;

      // The first NEW position is at t=motion_planner._prev_step_count * dt
      // I added 8 steps to solve a glitch where the car would always flip 180
      // and back to its position before continuing. This is ok since we are
      // teleporting all the way to the end of the trajectory, and that means
      // that we will always start a new one, which is garanteed to be more than
      // 8 steps long.
      double time_step = (motion_planner._prev_step_count + 10) * dt;

      // Get the first ego position (x,y,z) at time t=dt
      ego_curr_state.location.x =
          utils::evaluate(motion_planner._best_trajectory.x_coeff, time_step);
      ego_curr_state.location.y =
          utils::evaluate(motion_planner._best_trajectory.y_coeff, time_step);
      ego_curr_state.location.z =
          utils::evaluate(motion_planner._best_trajectory.z_coeff, time_step);

      // Let's get the next time step. For speed out of the teleportation and
      // visualization, you can try dt, 2*dt, 3*d, etc which will move the care
      // to next position after dt, 2*dt, 3*dt seconds have pased, respectively
      time_step += dt;
      while (time_step <= motion_planner._best_trajectory.duration) {
        // Get Next time step location
        ego_next_state.location.x =
            utils::evaluate(motion_planner._best_trajectory.x_coeff, time_step);
        ego_next_state.location.y =
            utils::evaluate(motion_planner._best_trajectory.y_coeff, time_step);
        ego_next_state.location.z =
            utils::evaluate(motion_planner._best_trajectory.z_coeff, time_step);

        // MJT does not give us yaw or pitch. Let's calcualte the yaw (heading)
        // and the pitch (in case we go on a hill, so the car dies not climb
        // totally horizontal)
        double delta_x;
        double delta_y;
        double delta_z;
        double new_yaw;
        double new_pitch;
        if (time_step + dt <= motion_planner._best_trajectory.duration) {
          delta_x = ego_next_state.location.x - ego_curr_state.location.x;
          delta_y = ego_next_state.location.y - ego_curr_state.location.y;
          delta_z = ego_next_state.location.z - ego_curr_state.location.z;

        } else {
          delta_x = ego_curr_state.location.x - ego_prev_state.location.x;
          delta_y = ego_curr_state.location.y - ego_prev_state.location.y;
          delta_z = ego_curr_state.location.z - ego_prev_state.location.z;
        }
        new_yaw = std::atan2(delta_y, delta_x);

        auto xy = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        new_pitch = std::atan2(delta_z, xy);

        ego_curr_state.rotation.yaw = rad2deg(new_yaw);
        ego_curr_state.rotation.pitch = rad2deg(new_pitch);

        LOG(INFO) << "MOVING TO x: " << ego_curr_state.location.x
                  << " y: " << ego_curr_state.location.y
                  << " z: " << ego_curr_state.location.z
                  << " yaw: " << deg2rad(ego_curr_state.rotation.yaw)
                  << " pitch: " << deg2rad(ego_curr_state.rotation.pitch)
                  << "delta_x: " << delta_x << " delta_y: " << delta_y
                  << " delta_z: " << delta_z;

        // Teleport ego car to this position and rotation
        ego_vehicle->SetTransform(ego_curr_state);

        // Move spectator so we can see the ego_vehicle from the simulator
        // window.
        auto spectator_transform = ego_curr_state;
        spectator_transform.location -=
            10.0f * spectator_transform.GetForwardVector();
        spectator_transform.location.z += 5.0f;
        spectator_transform.rotation.pitch = -15.0f;
        spectator->SetTransform(spectator_transform);

        // Give some time to CARLA to move and update all the world
        std::this_thread::sleep_for(0.1s);

        // Let's keep track on what step of the trajectory we are.
        ++motion_planner._prev_step_count;

        ego_prev_state = ego_curr_state;
        ego_curr_state = ego_next_state;

        time_step += dt;
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
  google::SetLogDestination(google::GLOG_INFO,
                            "/home/munir/carla/nd013-c5-planning-refresh/"
                            "project/solution/bin/logs.log");

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