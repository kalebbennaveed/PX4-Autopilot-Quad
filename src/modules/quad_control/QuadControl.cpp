#include "QuadControl.hpp"

using namespace matrix;

QuadControl::QuadControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {

  // initializer
  parameters_update();
}

QuadControl::~QuadControl() { perf_free(_cycle_perf); }

bool QuadControl::init() {
  if (!_ang_vel_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }

  _timestamp_last_loop = hrt_absolute_time();
  ScheduleNow();

  return true;
}

void QuadControl::parameters_update() {
  // check for parameter updates
  if (_parameter_update_sub.updated()) {
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);

    ModuleParams::updateParams();

    PX4_WARN("updateParams");

    // update parameters in controllers
    _controller.set_gains(
        _param_quad_kx.get(), _param_quad_kv.get(), _param_quad_ki.get(),
        _param_quad_kR.get(), _param_quad_kOmega.get(), _param_quad_m.get(),
        _param_quad_Jxx.get(), _param_quad_Jyy.get(), _param_quad_Jzz.get());

    // update mixer gains
    _mixer.set_thrust_coeff(_param_quad_kThrust.get());
    _mixer.set_torque_coeff(_param_quad_kTorque.get());
    _mixer.set_omega_max(_param_quad_omegaMax.get());

    // set all the mixer geometry
    _mixer.set_rotor(
        0, {_param_quad_rot1_pos_x.get(), _param_quad_rot1_pos_y.get(), 0}, -1);
    _mixer.set_rotor(
        1, {_param_quad_rot2_pos_x.get(), _param_quad_rot2_pos_y.get(), 0}, -1);
    _mixer.set_rotor(
        2, {_param_quad_rot3_pos_x.get(), _param_quad_rot3_pos_y.get(), 0}, 1);
    _mixer.set_rotor(
        3, {_param_quad_rot4_pos_x.get(), _param_quad_rot4_pos_y.get(), 0}, 1);

    // update the mixer G matrix
    _mixer.construct_G_matrix();

   // update the esc nonlinearity
    _mixer.set_esc_nonlinearity(_param_quad_esc_nonlinearity.get());

    _land_speed = _param_quad_land_speed.get();
  }
}

void QuadControl::Run() {

  if (should_exit()) {
    _ang_vel_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_cycle_perf);

  if (!_ang_vel_sub.updated()) {
    perf_end(_cycle_perf);
    return;
  }

  parameters_update();

  // do things only if ang_vel updated:

  // grab commander status
  if (_commander_status_sub.update(&_commander_status)) {
    _init_commander_status = true;

    _armed = _commander_status.state != commander_status_s::STATE_DISARMED;
  }

  if (!_armed) {
    float v = 0;
    _controller.reset_integral();
    Vector4f cmd(v, v, v, v);
    publish_cmd(cmd);
    perf_end(_cycle_perf);
    return;
  }

  // update other subscribers
  if (_ang_vel_sub.update(&_state_ang_vel)) {
    _controller.update_state_Omega(_state_ang_vel);
    _init_state_Omega = true;
  }

  if (_local_pos_sub.update(&_state_pos)) {
    _controller.update_state_pos(_state_pos);

    // if the mode is not landing, update the landing state
    if (_init_commander_status &&
        _commander_status.state != commander_status_s::STATE_LAND) {
      _local_pos_sub.copy(&_start_landing_state);
      _last_timestamp_land_started = hrt_absolute_time();
    }
    _init_state_pos = true;
  }

  if (_att_sub.update(&_state_att)) {
    _controller.update_state_attitude(_state_att);
    _init_state_att = true;
  }

  if (_trajectory_setpoint_sub.update(&_setpoint)) {
    _controller.update_setpoint(_setpoint);
    _init_setpoint = true;
  }

  _initialized = _init_state_Omega && _init_state_pos && _init_state_att &&
                 _init_setpoint && _init_commander_status;

  if (!_initialized) {
    _controller.reset_integral();
    float v = 0.05;
    Vector4f cmd(v, v, v, v);
    publish_cmd(cmd);
    PX4_WARN("ARMED but not INITIALIZED");
    perf_end(_cycle_perf);
    return;
  }

  // if in armed mode, publish the min PWM
  if (_commander_status.state == commander_status_s::STATE_ARMED) {
    _controller.reset_integral();
    float v = 0.05;
    Vector4f cmd(v, v, v, v);
    publish_cmd(cmd);
    perf_end(_cycle_perf);
    return;
  }

  // if in land mode, modify the setpoint
  if (_commander_status.state == commander_status_s::STATE_LAND) {

    // initialize setpoint to 0
    for (size_t i = 0; i < 3; i++) {
      _setpoint.velocity[i] = 0;
      _setpoint.acceleration[i] = 0;
      _setpoint.jerk[i] = 0;
    }
    _setpoint.yawspeed = 0;

    float t =
        (float)(1e-6) * (float)hrt_elapsed_time(&_last_timestamp_land_started);

    _setpoint.position[0] = _start_landing_state.x;
    _setpoint.position[1] = _start_landing_state.y;
    _setpoint.position[2] = _start_landing_state.z + _land_speed * t;
    _setpoint.yaw = _start_landing_state.heading;
    _setpoint.velocity[2] = _land_speed;
    _controller.update_setpoint(_setpoint);
  }

  // if the code is here, it is in OFFBOARD MODE and with the correct setpoint

  if (_setpoint.raw_mode == false) {
    // run controller
    _controller.run();

    // get thrust and torque command
    float thrust_cmd = _controller.get_thrust_cmd();
    thrust_cmd = (thrust_cmd < 0) ? 0 : thrust_cmd;
    Vector3f torque_cmd = _controller.get_torque_cmd().zero_if_nan();

    // do the mixing
    Vector4f cmd = _mixer.mix(thrust_cmd, torque_cmd);

    // publish
    publish_cmd(cmd);

  } else {
    PX4_WARN("IN RAW MOTOR MODE");

    // copy from the _setpoint msg
    Vector4f motor_cmd(_setpoint.cmd[0], _setpoint.cmd[1], _setpoint.cmd[2],
                 _setpoint.cmd[3]);

    // publish
    publish_cmd(motor_cmd);
  }

  perf_end(_cycle_perf);

  return;
}

void QuadControl::publish_cmd(Vector4f cmd) {
{
  // publish for sitl
  actuator_outputs_s msg;
  msg.timestamp = hrt_absolute_time();
  for (size_t i = 0; i < 4; i++) {
    msg.output[i] = cmd(i);
  }
  _actuator_outputs_sim_pub.publish(msg);
}
{ 
 // publish for hardware
  actuator_motors_s msg;
  msg.timestamp = hrt_absolute_time();
  msg.reversible_flags = 0; // no motors are reversible
  for (size_t i = 0; i < 4; i++) {
    msg.control[i] = cmd(i);
  }
 _actuator_motors_pub.publish(msg);

 
}
}

int QuadControl::task_spawn(int argc, char *argv[]) {

  QuadControl *instance = new QuadControl();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}


int QuadControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int QuadControl::print_usage(const char *reason) {

  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Quadrotor controller
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("quad_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int quad_control_main(int argc, char *argv[]) {
  return QuadControl::main(argc, argv);
}
