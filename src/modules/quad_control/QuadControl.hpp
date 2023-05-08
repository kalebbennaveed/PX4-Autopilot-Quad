#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/commander_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "Controllers/GeometricController.hpp"
#include "Controllers/IndiGeometricController.hpp"
#include "Controllers/MixerLinear.hpp"
#include "Controllers/MixerQuadratic.hpp"

using namespace time_literals;
using namespace matrix;

class QuadControl : public ModuleBase<QuadControl>,
                    public ModuleParams,
                    public px4::WorkItem {

public:
  QuadControl();
  ~QuadControl() override;

  static int task_spawn(int argc, char *argv[]);
  static int custom_command(int argc, char *argv[]);
  static int print_usage(const char *reason = nullptr);
  void handle_motor_test(int ind);
  void handle_motor_test_stop();

private:
  bool init();
  void Run() override;
  void parameters_update();
  void publish_cmd(Vector4f pwm_cmd);

  // Publications
  uORB::Publication<actuator_motors_s> _actuator_motors_pub{
      ORB_ID(actuator_motors)};
  uORB::Publication<actuator_outputs_s> _actuator_outputs_pub{
      ORB_ID(actuator_outputs)};
  uORB::Publication<actuator_outputs_s> _actuator_outputs_sim_pub{
      ORB_ID(actuator_outputs_sim)};

  // Subscriptions
  uORB::Subscription _commander_status_sub{ORB_ID(commander_status)};
  uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
  uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
  uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
  uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
  uORB::Subscription _acc_sub{ORB_ID(vehicle_acceleration)};
  uORB::SubscriptionCallbackWorkItem _ang_vel_sub{
      this, ORB_ID(vehicle_angular_velocity)};
  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};

  // Private Variables:
  vehicle_status_s _vehicle_status;
  vehicle_local_position_s _state_pos;
  vehicle_local_position_s _start_landing_state;
  vehicle_attitude_s _state_att;
  vehicle_acceleration_s _state_acc;
  vehicle_angular_velocity_s _state_ang_vel;
  trajectory_setpoint_s _setpoint;
  commander_status_s _commander_status;

  GeometricController _controller;
  MixerQuadratic _mixer;

  float _land_speed = 0.2f;

  bool _armed = false;
  bool _initialized = false;
  bool _init_state_Omega = false;
  bool _init_state_pos = false;
  bool _init_state_att = false;
  bool _init_state_acc = false;
  bool _init_setpoint = false;
  bool _init_commander_status = false;

  hrt_abstime _timestamp_last_loop{0};
  hrt_abstime _last_timestamp_land_started{0};
  perf_counter_t _cycle_perf{
      perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle time")};

  DEFINE_PARAMETERS(
      (ParamFloat<px4::params::QUAD_KX>)_param_quad_kx,
      (ParamFloat<px4::params::QUAD_KV>)_param_quad_kv,
      (ParamFloat<px4::params::QUAD_KI>)_param_quad_ki,
      (ParamFloat<px4::params::QUAD_KR>)_param_quad_kR,
      (ParamFloat<px4::params::QUAD_KOMEGA>)_param_quad_kOmega,
      (ParamFloat<px4::params::QUAD_M>)_param_quad_m,
      (ParamFloat<px4::params::QUAD_JXX>)_param_quad_Jxx,
      (ParamFloat<px4::params::QUAD_JYY>)_param_quad_Jyy,
      (ParamFloat<px4::params::QUAD_JZZ>)_param_quad_Jzz,
      (ParamFloat<px4::params::QUAD_KTHRUST>)_param_quad_kThrust,
      (ParamFloat<px4::params::QUAD_KTORQUE>)_param_quad_kTorque,
      (ParamFloat<px4::params::QUAD_OMEGA_MAX>)_param_quad_omegaMax,
      (ParamFloat<px4::params::QUAD_ESC_NONLIN>) _param_quad_esc_nonlinearity,
      (ParamFloat<px4::params::QUAD_LAND_SPEED>)_param_quad_land_speed,
      (ParamFloat<px4::params::QUAD_ROT1_POSX>)_param_quad_rot1_pos_x,
      (ParamFloat<px4::params::QUAD_ROT2_POSX>)_param_quad_rot2_pos_x,
      (ParamFloat<px4::params::QUAD_ROT3_POSX>)_param_quad_rot3_pos_x,
      (ParamFloat<px4::params::QUAD_ROT4_POSX>)_param_quad_rot4_pos_x,
      (ParamFloat<px4::params::QUAD_ROT1_POSY>)_param_quad_rot1_pos_y,
      (ParamFloat<px4::params::QUAD_ROT2_POSY>)_param_quad_rot2_pos_y,
      (ParamFloat<px4::params::QUAD_ROT3_POSY>)_param_quad_rot3_pos_y,
      (ParamFloat<px4::params::QUAD_ROT4_POSY>)_param_quad_rot4_pos_y)
};
