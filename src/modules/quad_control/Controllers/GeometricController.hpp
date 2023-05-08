#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
// #include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class GeometricController {

public:
  GeometricController();

  void set_gains(float _kx = 1.0f, float _kv = 2.0f, float _kI = 0.01f,
                 float _kR = 0.35f, float _kOmega = 0.15f, float _m = 1.5f,
                 float _Jxx = 0.005, float _Jyy = 0.005, float _Jzz = 0.009);
  void reset_integral();
  void update_state_pos(vehicle_local_position_s pos);
  void update_state_attitude(vehicle_attitude_s att);
  void update_state_Omega(vehicle_angular_velocity_s ang_vel);
  void update_setpoint(trajectory_setpoint_s sp);
  void run();

  float get_thrust_cmd();
  Vector3f get_torque_cmd();

private:
  // PARAMETERS
  float kx, kv, ki, kR, kOmega;
  float m, g;
  SquareMatrix<float, 3> J;

  // STATE
  Vector3f x, v, Omega, ei;
  Dcmf R;

  // SETPOINT
  Vector3f x_ref, v_ref, a_ref, j_ref, s_ref, b1_ref, Omega_ref, alpha_ref;
  float yaw_ref, yaw_vel_ref, yaw_acc_ref;

  // results
  float thrust_cmd;
  Vector3f torque_cmd;
};
