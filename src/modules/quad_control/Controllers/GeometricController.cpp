
// Devansh Agrawal
// April 2023

#include "GeometricController.hpp"
#include "DiffFlat.hpp"

using namespace matrix;

GeometricController::GeometricController() {
  set_gains();
  reset_integral();
}

void GeometricController::set_gains(float _kx, float _kv, float _ki, float _kR,
                                    float _kOmega, float _m, float _Jxx,
                                    float _Jyy, float _Jzz) {

  g = 9.81f;

  kx = _kx;         // 1.0f;
  kv = _kv;         // 2.0f;
  ki = _ki;         // 0.05f;
  kR = _kR;         // 0.35f;
  kOmega = _kOmega; // 0.15f;

  m = _m; // 1.5f;

  J.setZero();
  J(0, 0) = _Jxx; // 0.005;
  J(1, 1) = _Jyy; // 0.005;
  J(2, 2) = _Jzz; // 0.009;
}

void GeometricController::reset_integral() { ei.setZero(); }

void GeometricController::update_state_pos(vehicle_local_position_s pos) {
  // grab pos and vel
  x(0) = pos.x;
  x(1) = pos.y;
  x(2) = pos.z;

  v(0) = pos.vx;
  v(1) = pos.vy;
  v(2) = pos.vz;
}

void GeometricController::update_state_attitude(vehicle_attitude_s att) {
  // grab rotation from body frame to earth frame
  R = Dcmf(Quatf(att.q));
}

void GeometricController::update_state_Omega(
    vehicle_angular_velocity_s ang_vel) {
  for (size_t i = 0; i < 3; i++) {
    Omega(i) = ang_vel.xyz[i];
  }
}

void GeometricController::update_setpoint(trajectory_setpoint_s sp) {

  // these are in FRD frame
  for (size_t i = 0; i < 3; i++) {
    x_ref(i) = sp.position[i];
    v_ref(i) = sp.velocity[i];
    a_ref(i) = sp.acceleration[i];
    j_ref(i) = sp.jerk[i];
    s_ref(i) = 0.0f;
  }

  yaw_ref = -sp.yaw;
  yaw_vel_ref = -sp.yawspeed;
  yaw_acc_ref = 0.0f;

  // do the flat state to trajectory setpoint conversion
  flat_state_FRD_to_quad_state_FRD(b1_ref, Omega_ref, alpha_ref, a_ref, j_ref,
                                   s_ref, yaw_ref, yaw_vel_ref, yaw_acc_ref);
}

// Run the controller
// control input is stored in omega
void GeometricController::run() {

  // assuming FRD frame for everything

  const Vector3f e3(0, 0, 1);

  Vector3f ex = (x - x_ref).zero_if_nan();
  Vector3f ev = v - v_ref;

  // prevent too large ex:
  if (ex.norm() > 2) {
    ex = 2.0f * ex / ex.norm();
  }

  // prevent too large ev:
  if (ev.norm() > 5) {
    ev = 5.0f * ev / ev.norm();
  }

  // increment the integral error
  ei += ex;
  // prevent windup
  for (size_t i = 0; i < 3; i++) {
    ei(i) = (ei(i) > 2.0f / ki) ? 2.0f / ki : ei(i);
    ei(i) = (ei(i) < -2.0f / ki) ? -2.0f / ki : ei(i);
  }
  Vector3f thrust = -kx * ex - kv * ev - ki * ei - m * g * e3 + m * a_ref;

  Vector3f b3d = -thrust.unit();
  Vector3f b2d = (b3d.cross(b1_ref)).unit();
  Vector3f b1d = (b2d.cross(b3d)).unit();

  // construct desired rotation matrix
  Dcmf Rd;
  for (size_t i = 0; i < 3; i++) {
    Rd(i, 0) = b1d(i);
    Rd(i, 1) = b2d(i);
    Rd(i, 2) = b3d(i);
  }

  Vector3f eR = 0.5f * (Dcmf(Rd.T() * R - R.T() * Rd)).vee();
  Vector3f eOmega = Omega - R.T() * Rd * Omega_ref;

  thrust_cmd = -thrust.dot(R * e3);

  torque_cmd =
      -kR * eR - kOmega * eOmega + Omega.cross(J * Omega) -
      J * (Omega.hat() * R.T() * Rd * Omega_ref - R.T() * Rd * alpha_ref);

  return;
}

float GeometricController::get_thrust_cmd() { return thrust_cmd; }

Vector3f GeometricController::get_torque_cmd() { return torque_cmd; }
