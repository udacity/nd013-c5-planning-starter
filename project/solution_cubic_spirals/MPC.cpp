/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#include "MPC.h"

#include <cfloat>
#include <iostream>

#include <carla/client/Client.h>
#include <glog/logging.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/QR"

#include "vehicle_dynamic_model.h"

namespace cc = carla::client;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using CppAD::AD;
using namespace Eigen;

// TODO: Set the timestep length and duration
const uint32_t N = 10;  // prediction Horizon. NOTE: MUST BE SHORTER THAN THE
                        // NUMBER OF TRAJECTORY POINTS!!
// const double dt = 0.1;  // process time step
// const double T = N * dt;  // This is the Prediction Horizon in seconds.
// const double latency = 0.05;  // 50 ms

// The solver takes all the state variables and actuator
// variables in a single vector. On the lectured we've seen how this structure
// is. Here, we establish when one variable starts and another ends to be able
// to address its indexes in an easy way.
uint32_t x_start = 0;
uint32_t y_start = x_start + N;
uint32_t yaw_start = y_start + N;
uint32_t v_start = yaw_start + N;

// actuators
uint32_t steer_start = v_start + N;
uint32_t throttle_start = steer_start + N - 1;

// define the WEIGTHS that we will use to quantify how "costly" (bad) are each
// component of the COST function Basically HOW important is each element of the
// COST function: For instance, it's very important that cte remains close to 0
// but also it's veru important to make sure that the changes in commands
// (steering and throattle) are smooth. For more explanations look below on the
// COST function construntion

const double W_x_err = 0.01;
const double W_y_err = 0.01;
const double W_yaw_err = 0.5;
const double W_vel_err = 0.5;

const double W_steer_use = 0.01;
const double W_throttle_use = 0.01;

const double W_dSteer = 0.01;
const double W_dthrottle = 0.01;

class FG_eval {
 private:
  // Spiral path + velocity profile from the motion planner
  std::vector<TrajectoryPoint> _trajectory;
  // We should get the specific Vehicle details. Waiting for CARLA people to
  // respond
  VehicleDynamicModel _vehicle_dynamic_model;
  SharedPtr<carla::client::Vehicle> _vehicle;

 public:
  // Contructor
  FG_eval(const std::vector<TrajectoryPoint>& trajectory,
          const SharedPtr<carla::client::Vehicle> vehicle) {
    this->_trajectory = trajectory;
    this->_vehicle = vehicle;
    this->_vehicle_dynamic_model.load_vehicle_data(_vehicle);

    // Get some basic Physical data from the vehicle
    auto physics_control = _vehicle->GetPhysicsControl();
    // for (auto wheel : physics_control.wheels) {
    //   LOG(INFO) << "Max Steer: " << wheel.max_steer_angle;
    // }
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators) NOTE: You'll probably go back and forth
    // between this function and the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Cost function
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.

    // Minimize the use of actuators.
    for (uint32_t t = 0; t < N - 1; t++) {
      fg[0] += W_steer_use * CppAD::pow(vars[steer_start + t], 2);
      fg[0] += W_throttle_use * CppAD::pow(vars[throttle_start + t], 2);
    }

    // Minimize the value gap between sequential actuations. (This is actually
    // to guarantee a min "snap" trajectory) We could try to use even deeper
    // derivatives (min Jerk trajectories), but for now we can see how this
    // performs.
    for (uint32_t t = 0; t < N - 2; t++) {
      fg[0] += W_dSteer *
               CppAD::pow(vars[steer_start + t + 1] - vars[steer_start + t], 2);
      fg[0] += W_dthrottle *
               CppAD::pow(
                   vars[throttle_start + t + 1] - vars[throttle_start + t], 2);
    }

    /*
    By definition, fg has the following structure:
            fg[0] - is the accumulated COST
            1 to N*6 - The initial values

    */

    // fix the initial state as a constraint
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];

    // The rest of the constraints
    // Before we start defining the constraints, we need to recall that, as a
    // discrete model, all the constraints at time "t+1" depend on the values at
    // "t" AND also that for simplicity we put all the constraints of the form
    // XX(t+1) = F(X(t))
    // as
    // F(X(t)) - XX(t+1) = 0 or XX(t+1) - F(X(t)) = 0
    // Therefore, we will start collecting all the actual (t+1) and previous (t)
    // values
    for (uint32_t t = 0; t < N - 1; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t + 1];
      AD<double> y1 = vars[y_start + t + 1];
      AD<double> yaw1 = vars[yaw_start + t + 1];
      AD<double> v1 = vars[v_start + t + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + t];
      AD<double> y0 = vars[y_start + t];
      AD<double> yaw0 = vars[yaw_start + t];
      AD<double> v0 = vars[v_start + t];

      // Only consider the actuation at time t.
      AD<double> steer0 =
          vars[steer_start + t] * _vehicle_dynamic_model.max_steer_angle;
      AD<double> throttle0 = vars[throttle_start + t];

      AD<double> a0 = _vehicle_dynamic_model.get_accel(throttle0, v0, _vehicle);
      // LOG(INFO) << "Th0: " << throttle0;
      // LOG(INFO) << "v0: " << v0;
      // LOG(INFO) << "a0: " << a0;

      double DT =
          _trajectory[t + 1].relative_time - _trajectory[t].relative_time;

      // constraint with the kinematic model: Rear Axle Bycicle Model
      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
      fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
      fg[2 + yaw_start + t] = yaw1 - (yaw0 + v0 * CppAD::tan(steer0) / WB * DT);
      fg[2 + v_start + t] = v1 - (v0 + a0 * DT);

      // Additional cost based on the ERRORS w.r.t. the ref trajectory
      fg[0] += W_x_err * CppAD::pow(_trajectory[t + 1].path_point.x -
                                        (x0 + v0 * CppAD::cos(yaw0) * DT),
                                    2);
      fg[0] += W_y_err * CppAD::pow(_trajectory[t + 1].path_point.y -
                                        (y0 + v0 * CppAD::sin(yaw0) * DT),
                                    2);
      fg[0] +=
          W_yaw_err * CppAD::pow(_trajectory[t + 1].path_point.theta -
                                     (yaw0 + v0 * CppAD::tan(steer0) / WB * DT),
                                 2);
      fg[0] += W_vel_err * CppAD::pow(_trajectory[t + 1].v - (v0 + a0 * DT), 2);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(SharedPtr<cc::Vehicle> vehicle) {
  _vehicle = vehicle;
  _max_steer_angle = utils::deg2rad(
      vehicle->GetPhysicsControl().wheels[0].max_steer_angle / 2.0);
}

MPC::~MPC() {}

std::vector<double> MPC::Solve(
    const std::vector<TrajectoryPoint>& trajectory,
    const std::chrono::high_resolution_clock::time_point& start_time) {
  //LOG(INFO) << "MPC- Solve started";

  //LOG(INFO) << "Preparing data";

  // The MPC optimization is easier to setup and also faster if we tranform all
  // involved parts into a Ego-center referance frame
  // auto trajectory_wrt_ego = trajectory_into_ego_ref(trajectory);
  auto ego_transform =
      boost::static_pointer_cast<cc::Actor>(_vehicle)->GetTransform();
  auto ego_x = ego_transform.location.x;
  auto ego_y = ego_transform.location.y;
  auto ego_yaw = utils::deg2rad(ego_transform.rotation.yaw);

  // Get the speed
  auto ego_velocity_vec = _vehicle->GetVelocity();
  auto ego_velocity_mag = utils::magnitude(ego_velocity_vec);
  //LOG(INFO) << "ego v(m/s): " << ego_velocity_mag;

  // Let's build the state of the systems that will be used to feed the MPC.
  // state = x, y, yaw, velo
  // Rembember that px, py and yaw wrt car are all 0.
  // For clarity

  double x = ego_x;  // 0.0;    // Always 0 since we moved to the Car Ref System
  double y = ego_y;  // 0.0;    // Always 0 since we moved to the Car Ref System
  double yaw = ego_yaw;
  double v = ego_velocity_mag;

  // TODO: How do you want to handle latency. What's the actual (new) initial
  // state
  auto steer_angle = _vehicle->GetControl().steer * _max_steer_angle;
  auto ego_accel_vect = _vehicle->GetAcceleration();
  auto accel = utils::magnitude(ego_accel_vect);
  long long latency =
      std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::high_resolution_clock::now() - start_time)
          .count();
  //LOG(INFO) << "latency(s): " << latency;
  x += v * latency + (0.5 * accel * latency * latency);
  y += 0;
  yaw += -v * CppAD::tan(steer_angle) / WB * latency;
  v += accel * latency;

  const int StateSize = 4;
  const int NumActuators = 2;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // In "N" timesteps => "N - 1" actuations

  const uint32_t n_vars = N * StateSize + (N - 1) * NumActuators;

  // TODO: Set the number of constraints
  /* In theory, and since each state is 1 dimention, it needs to be bounded:
     Lower anf higher. We will assume that this variable is actually refering to
     the number of VARIBALES (from the State) that will have cosntrains. Later
     we will apply the the corresponding Low and high boundary contrsiant.
  */
  const uint32_t n_constraints = N * StateSize;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  // We will set the initial state after we decide what to do with the
  // latency

  Dvector vars(n_vars);
  for (uint32_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Initial State:
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[yaw_start] = yaw;
  vars[v_start] = v;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators (x,y,yaw,v,cte,err_yaw) upper and lowerlimits to the
  // max negative and positive values. We can refine these limits but for
  // simplicity we do this for now.
  for (uint32_t i = 0; i < steer_start; i++) {
    vars_lowerbound[i] = DBL_MIN;
    vars_upperbound[i] = DBL_MAX;
  }

  // The upper and lower limits for Steering IS -1 to +1.
  for (uint32_t i = steer_start; i < throttle_start; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // The upper and lower limits for Throttle IS -1 to 1.
  for (uint32_t i = throttle_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint32_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Initial state should have smae upper and lower bounds since it will NOT
  // change
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[yaw_start] = yaw;
  constraints_lowerbound[v_start] = v;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[yaw_start] = yaw;
  constraints_upperbound[v_start] = v;

  // object that computes objective and constraints
  FG_eval fg_eval(trajectory, _vehicle);  // trajectory_wrt_ego

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Integer max_iter      100\n";
  options += "Numeric max_cpu_time          0.2\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  //bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //LOG(INFO) << "IPOPT solve result ok? " << ok;

  // Cost
  //auto cost = solution.obj_value;
  //LOG(INFO) << "Cost " << cost;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x_t = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result;

  result.push_back(solution.x[steer_start]);
  result.push_back(solution.x[throttle_start]);

  // This is done for visualization. (to show a MPC trajectory)
  // We will add all the N step MPC {x,y} solutions.
  for (uint32_t i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}

std::vector<TrajectoryPoint> MPC::trajectory_into_ego_ref(
    const std::vector<TrajectoryPoint>& trajectory) {
  //LOG(INFO) << "Waypoints_into_ego_ref";
  auto ego_transform =
      boost::static_pointer_cast<cc::Actor>(_vehicle)->GetTransform();
  auto ego_x = ego_transform.location.x;
  auto ego_y = ego_transform.location.y;
  auto ego_yaw = utils::deg2rad(ego_transform.rotation.yaw);
  // auto ego_x{220.0};
  // auto ego_y{-5.70};
  // auto ego_yaw{-3.121};

  double cos_yaw = std::cos(-ego_yaw);
  double sin_yaw = std::sin(-ego_yaw);

  // Let's start making a copy, since we are only changing x, y and yaw. The
  // rest will be preserved in both reference frames
  auto transformed_trajectory = trajectory;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    auto traj_point = trajectory[i];
    double dx = (traj_point.path_point.x - ego_x);
    double dy = (traj_point.path_point.y - ego_y);
    transformed_trajectory[i].path_point.x = cos_yaw * dx - sin_yaw * dy;
    transformed_trajectory[i].path_point.y = sin_yaw * dx + cos_yaw * dy;
    transformed_trajectory[i].path_point.theta =
        traj_point.path_point.theta - ego_yaw;
  }

  return transformed_trajectory;
}