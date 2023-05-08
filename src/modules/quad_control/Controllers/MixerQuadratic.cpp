#include "MixerQuadratic.hpp"

MixerQuadratic::MixerQuadratic() {

  _k_thrust = 5.84;              // N / (krad/s)^2
  _k_torque = 0.06f * _k_thrust; // Nm / (krad/s)^2
  _k_omega_max = 1.1;            // krad/s
  _k_alpha =
      0.0; // assumes linear map from omega/omega_max to actuator_cmd in [0, 1]

  // default: assumes the quadrotor is arranged as follows:
  //     +x
  //      ^
  //      |
  //      |
  //   2     0
  //      x       --> +y
  //   1     3
  //
  // where 1 is spinning ccw
  //
  // Use an FRD frame
  float L = 0.33 / 2.0; // arm_length
  float LX = L / sqrtf(2.0f);
  float LY = L / sqrtf(2.0f);

  Vector3f rotor_pos_0(LX, LY, 0);
  Vector3f rotor_pos_1(-LX, -LY, 0);
  Vector3f rotor_pos_2(LX, -LY, 0);
  Vector3f rotor_pos_3(-LX, LY, 0);

  set_rotor(0, rotor_pos_0, -1);
  set_rotor(1, rotor_pos_1, -1);
  set_rotor(2, rotor_pos_2, 1);
  set_rotor(3, rotor_pos_3, 1);

  construct_G_matrix();
}

void MixerQuadratic::set_thrust_coeff(float k) { _k_thrust = k; }

void MixerQuadratic::set_torque_coeff(float k) { _k_torque = k; }

void MixerQuadratic::set_omega_max(float w) { _k_omega_max = w; }

void MixerQuadratic::set_esc_nonlinearity(float alpha) { _k_alpha = alpha; }

SquareMatrix<float, 4> MixerQuadratic::get_G_matrix() { return _invG.I(); }

// dir: +1 if it produces torque in +z direction (i.e., down). -1 else
void MixerQuadratic::set_rotor(size_t ind, Vector3f pos, int dir) {
  rotor_pos[ind] = pos;
  rotor_dir[ind] = dir;
}

void MixerQuadratic::construct_G_matrix() {

  // construct the G, Ginv matrix,
  //
  // assume relationship is
  // [thrust, torque] = G * omega^2
  // w = omega / omega_max;
  // cmd = (1-alpha) w + alpha w^2;
  //
  // where
  //   thrust is in N
  //   torque is in Nm
  //   omega is in krad/s
  //   w is in [0, 1]
  //   cmd is in [0, 1]
  //   alpha is in [0, 1]
  //
  // the parameter alpha controls how nonlinear the esc mapping is

  Vector3f iz(0, 0, 1);

  SquareMatrix<float, 4> G;

  for (size_t ind = 0; ind < 4; ind++) {
    G(0, ind) = _k_thrust;
  }

  for (size_t ind = 0; ind < 4; ind++) {
    Vector3f torque = -_k_thrust * rotor_pos[ind].cross(iz) -
                      (float)rotor_dir[ind] * Vector3f(0, 0, _k_torque);
    for (size_t j = 0; j < 3; j++) {
      G(j + 1, ind) = torque(j);
    }
  }

  // store inverse G matrix
  _invG = G.I();
}

Vector4f MixerQuadratic::mix(float thrust_cmd, Vector3f torque_cmd) {

  Vector4f fM;
  fM(0) = thrust_cmd;
  for (size_t i = 0; i < 3; i++) {
    fM(i + 1) = torque_cmd(i);
  }

  // do the inverse
  Vector4f omega_sq = _invG * fM;

  // get the omega required
  Vector4f omega;
  for (size_t i = 0; i < 4; i++) {
    omega(i) = (omega_sq(i) > 0) ? sqrt(omega_sq(i)) : 0;
    omega(i) = (omega(i) <= _k_omega_max) ? omega(i) : _k_omega_max;
  }

  // convert to CMD
  Vector4f cmd;
  for (size_t i = 0; i < 4; i++) {
    float w = omega(i) / _k_omega_max;
    cmd(i) = (1 - _k_alpha) * w + _k_alpha * w * w;
  }

  return cmd;
}
