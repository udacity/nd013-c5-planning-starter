/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file structs.h
 **/

#pragma once

#include <vector>

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"

using namespace std;

namespace cg = carla::geom;

enum Maneuver  // A.K.A State
{
  // Follow Lane - Maintain a constant speed = SpeedLimit-buffer in the same
  // lane
  FOLLOW_LANE,

  // Following - Disntance with Vehicle in front is < d_min
  // and we need to slow down/speed Up (calc accel) behind it until we can
  // Change lane or move at the desired speed
  FOLLOW_VEHICLE,

  DECEL_TO_STOP,

  STOPPED
};

struct Trajectory {
  Maneuver maneuver;
  vector<double> x_coeff;
  vector<double> y_coeff;
  vector<double> z_coeff;
  vector<double> yaw_coeff;
  vector<double> start_x;
  vector<double> start_y;
  vector<double> start_z;
  vector<double> start_yaw;
  vector<double> goal_x;
  vector<double> goal_y;
  vector<double> goal_z;
  vector<double> goal_yaw;
  double duration;  // maneuver estimated (also desired) duration
};

struct State {
  cg::Location location;
  cg::Rotation rotation;
  cg::Vector3D velocity;
  cg::Vector3D acceleration;
};

/*
Cubic Spirals are defined by their curvature as a function of their arc-length
"s".
Given the spiral (i.e curvature polynomio):
kappa(s) = a + b*s + c*s^2 + d*s^3
*/
struct CubicSpiral {
  double a;
  double b;
  double c;
  double d;
};

struct maneuver_params {
  int dir;              // directiion: +1(left), 0 (stay in lane), -1 (right)
  double target_x;      // target final s coordinate. Ego_s + maneuver len
  double target_speed;  // target end_speed of the maneuver
  double duration;      // target duration
};