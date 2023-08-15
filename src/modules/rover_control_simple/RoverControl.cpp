#include "RoverControl.hpp"

using namespace matrix;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);



// ===================== Old code ===========================
// README: In the new version I removed the performace counter and added the paramters_update; Following Dev's structure

// RoverPositionControl::RoverPositionControl() :
// 	ModuleParams(nullptr),
// 	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)


// 	/* performance counters */
// 	// Removing this for now
// 	_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle")) // TODO : do we even need these perf counters
// {
// }
// ==========================================================


RoverPositionControl::RoverPositionControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {

  // initializer
  parameters_update();
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
}

bool
RoverPositionControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_timestamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void RoverPositionControl::parameters_update()
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		PX4_WARN("updateParams"); // Add warning that params are updated

		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());

		// PID Controler Inits
		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl,
				   _param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());
		// TOD0: Add mixer
		// We have the thrust and torque and needs to be converted to the PWM

		// What is _land_speed ?
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	if (_control_mode.flag_control_manual_enabled) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
			float dt = math::constrain(hrt_elapsed_time(&_manual_setpoint_last_called) * 1e-6f,  0.0002f, 0.04f);

			if (!_control_mode.flag_control_climb_rate_enabled &&
			    !_control_mode.flag_control_offboard_enabled) {

				if (_control_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.roll_body = 0.0;
					_att_sp.pitch_body = 0.0;

					/* reset yaw setpoint to current position if needed */
					if (_reset_yaw_sp) {
						const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
						_manual_yaw_sp = vehicle_yaw;
						_reset_yaw_sp = false;

					} else {
						const float yaw_rate = math::radians(_param_gnd_man_y_max.get());
						_att_sp.yaw_sp_move_rate = _manual_control_setpoint.roll * yaw_rate;
						_manual_yaw_sp = wrap_pi(_manual_yaw_sp + _att_sp.yaw_sp_move_rate * dt);
					}

					_att_sp.yaw_body = _manual_yaw_sp;
					_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();


					_attitude_sp_pub.publish(_att_sp);

				} else {
					// Set heading from the manual roll input channel
					_yaw_control = _manual_control_setpoint.roll; // Nominally yaw: _manual_control_setpoint.yaw;
					// Set throttle from the manual throttle channel
					_throttle_control = (_manual_control_setpoint.throttle + 1.f) * .5f;
					_reset_yaw_sp = true;
				}

			} else {
				_reset_yaw_sp = true;
			}

			_manual_setpoint_last_called = hrt_absolute_time();
		}
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}
void
RoverPositionControl::control_local_position(const matrix::Vector2f &current_position,
				       const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity)
{
	// TODO: validity check

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) ) {

		/* AUTONOMOUS FLIGHT */
		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* current waypoint (the one currently heading for) */
		matrix::Vector2f curr_sp(_trajectory_setpoint.x, _trajectory_setpoint.y);
                matrix::Vector2f pos_error( current_position(0)-curr_sp(0), current_position(1)-curr_sp(1) );
		float dist_target = pos_error.norm();//sqrt( dist_sq );
		// PX4_WARN("Error pos: %f %f %f",(double)pos_error(0), (double)pos_error(1),(double)dist_target);

	        switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {
				if (dist_target < 0.1f){//(double)_param_nav_loiter_rad.get()) {
					_pos_ctrl_state = STOPPING;  // We are closer than loiter radius to waypoint, stop.

				} else {
					Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

					// Yaw Error
					// float current_yaw = 2.0f * atan2f( _vehicle_att.q[3], _vehicle_att.q[0] );
					float current_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
					float desired_yaw = atan2f( -pos_error(1), -pos_error(0) );
					float yaw_error = desired_yaw - current_yaw;

					// Clip yaw to -pi to pi
					if (yaw_error > M_PI_F){
						yaw_error = 2.0f * M_PI_F - yaw_error;
					}
					else if (yaw_error < -M_PI_F){
						yaw_error = 2.0f * M_PI_F + yaw_error;
					}

					// Design angular velocity: Always run attitude control
					float yaw_rate_sp =  _param_rover_k_yaw.get() * yaw_error;
					float x_vel_sp = 0.0f;

					// Design linear velocity
					if ( (yaw_error>0.2f) || (yaw_error<-0.2f)){  // rotate only
						x_vel_sp = 0.0f;
					}
					else{					      // rotate and move
						x_vel_sp = cos(yaw_error) * _param_rover_k_pos.get() * pos_error.norm();
					}

					control_velocity(current_velocity, current_angular_velocity, x_vel_sp, yaw_rate_sp);
				}
			}
			break;

		case STOPPING: {
				_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				// PX4_WARN("Stopping- dist target: %f", (double)dist_target);
				if (dist_target > 0) {
					_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				}
			}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

	}

}

void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity, const float desired_linear_x_velocity, const float desired_angular_z_velocity)
{
	// assume desired velocity in body frame: Unicycle type of control

	float dt = 0.01; // Using non zero value to a avoid division by zero

	// get body frame current velocity
	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));
	const float x_vel = vel(0);

	// get vehicle acceleration for derivative term
	// TODO: convert to body frame
	// const float x_acc = _vehicle_acceleration_sub.get().xyz[0];
	Vector3f acc_body = R_to_body * Vector3f( _vehicle_acceleration_sub.get().xyz[0], _vehicle_acceleration_sub.get().xyz[1], _vehicle_acceleration_sub.get().xyz[2] );
	const float x_acc = acc_body(0);

	// get Thrust command
	const float control_throttle =  _param_rover_vel_ff.get() * desired_linear_x_velocity +  pid_calculate(&_speed_ctrl, desired_linear_x_velocity, x_vel, x_acc, dt);

	//Constrain maximum throttle to mission throttle
	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -_param_throttle_max.get(), _param_throttle_max.get());

	// get angular torque command
	const float control_yaw = _param_rover_omg_ff.get() * desired_angular_z_velocity + pid_calculate(&_angular_speed_ctrl, desired_angular_z_velocity, current_angular_velocity(2), 0.0, dt);
	_act_controls.control[actuator_controls_s::INDEX_YAW] = math::constrain(control_yaw, -1.0f, 1.0f);
}

void
RoverPositionControl::control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
{
	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
	const Eulerf euler_sp = qe;

	float control_effort = euler_sp(2) / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	const float control_throttle = att_sp.thrust_body[0];

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =  math::constrain(control_throttle, 0.0f, 1.0f);

}

void
RoverPositionControl::Run()
{
	// *** Where is should_exit() defined ?
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

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;


	// grab commander status
	if (_commander_status_sub.update(&_commander_status)) {
	_init_commander_status = true;

	_armed = _commander_status.state != commander_status_s::STATE_DISARMED;
	}


	if (!_armed) {
	float v = 0;
	Vector4f cmd(v, v, v, v); // TODO: Need to change this
	publish_cmd(cmd);
	perf_end(_cycle_perf);
	return;
	}

	// update other subscribers
	if (_ang_vel_sub.update(&_state_ang_vel)) {

	// TODO: Add controller for rover with function update_state_Omega(_state_ang_vel)
	// _controller.update_state_Omega(_state_ang_vel);

	_init_state_Omega = true;
	}

	if (_local_pos_sub.update(&_state_pos)) {

	// TODO: Add controller for rover with function update_state_pos
	// _controller.update_state_pos(_state_pos); [OLD]

	// if the mode is not landing, update the landing state
	if (_init_commander_status &&
		_commander_status.state != commander_status_s::STATE_LAND) {
	_local_pos_sub.copy(&_start_landing_state);
	_last_timestamp_land_started = hrt_absolute_time();
	}
	_init_state_pos = true;
	}

	if (_trajectory_setpoint_sub.update(&_setpoint)) {
	// TODO: Add controller for rover with function update_setpoint(_setpoint)
	// _controller.update_setpoint(_setpoint); [OLD]
	_init_setpoint = true;
	}


	if (_att_sub.update(&_state_att)) {
	// TODO: Add controller for rover with function update_state_attitude(_state_att)
	// _controller.update_state_attitude(_state_att); [OLD]
	_init_state_att = true;
	}

	_initialized = _init_state_Omega && _init_state_pos && _init_state_att &&
			_init_setpoint && _init_commander_status;

	if (!_initialized) {
	float v = 0.05;
	Vector4f cmd(v, v, v, v); // TODO: Need to change this
	publish_cmd(cmd);
	PX4_WARN("ARMED but not INITIALIZED");
	perf_end(_cycle_perf);
	return;
	}

	// if in armed mode, publish the min PWM
	if (_commander_status.state == commander_status_s::STATE_ARMED) {
	float v = 0.05;
	Vector4f cmd(v, v, v, v);
	publish_cmd(cmd);
	perf_end(_cycle_perf);
	return;
	}



	// ================ TDOD: First define the setpoint and set to the prev so that robot stays at its current pos ? =====
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
	// =======================================================================================================

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
	// *** What is the RAW MOTOR Mode
	PX4_WARN("IN RAW MOTOR MODE");

	// copy from the _setpoint msg
	Vector4f motor_cmd(_setpoint.cmd[0], _setpoint.cmd[1], _setpoint.cmd[2],
			_setpoint.cmd[3]);

	// publish
	publish_cmd(motor_cmd);
	}

	perf_end(_cycle_perf);

	return;



	// ======================================= [OLD Code] =================================
	// if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

	// 	// PX4_WARN("Inside rover!!");
	// 	/* check vehicle control mode for changes to publication state */
	// 	vehicle_control_mode_poll();
	// 	attitude_setpoint_poll();
	// 	vehicle_attitude_poll();
	// 	manual_control_setpoint_poll();

	// 	_vehicle_acceleration_sub.update();

	// 	/* update parameters from storage */
	// 	parameters_update();

	// 	/* only run controller if position changed */
	// 	if (_local_pos_sub.update(&_local_pos)) {

	// 		// position global but velocity local?? ///////////////////////////////////////////////////////////////
	// 		matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy,  _local_pos.vz);
	// 		// matrix::Vector2d current_position(_global_pos.lat, _global_pos.lon);
	// 		matrix::Vector2f current_position(_local_pos.x, _local_pos.y);
	// 		matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy, _local_pos.vz);
	// 		matrix::Vector3f current_angular_velocity(angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]);

	// 		//Position Control Mode
	// 		if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
	// 			_trajectory_setpoint_sub.update(&_trajectory_setpoint);
	// 			// PX4_WARN("%f %f %f",(double)_trajectory_setpoint.x, (double)_trajectory_setpoint.y, (double)_trajectory_setpoint.z);
	// 			control_local_position(current_position, current_velocity, current_angular_velocity);
	// 			// PX4_WARN("")

	// 		// Velocity Control Mode
	// 		} else if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_velocity_enabled) {
	// 			_trajectory_setpoint_sub.update(&_trajectory_setpoint);
	// 			// control_velocity(current_velocity, current_angular_velocity);
	// 			Vector3f desired_velocity{_trajectory_setpoint.vx, _trajectory_setpoint.vy, _trajectory_setpoint.vz};
	// 			float desired_linear_x_velocity = desired_velocity(0);
	// 			float desired_angular_z_velocity = _trajectory_setpoint.yawspeed;
	// 			control_velocity(current_velocity, current_angular_velocity, desired_linear_x_velocity, desired_angular_z_velocity);
	// 		}
	// 	}

	// 	// Respond to an attitude update and run the attitude controller if enabled
	// 	if (_control_mode.flag_control_attitude_enabled
	// 	    && !_control_mode.flag_control_position_enabled
	// 	    && !_control_mode.flag_control_velocity_enabled) {
	// 		control_attitude(_vehicle_att, _att_sp);
	// 		// PX4_WARN("Attitude Mode");
	// 	}

	// 	/* Only publish if any of the proper modes are enabled */
	// 	if (_control_mode.flag_control_velocity_enabled ||
	// 	    _control_mode.flag_control_attitude_enabled ||
	// 	    _control_mode.flag_control_position_enabled ||
	// 	    _control_mode.flag_control_manual_enabled) {
	// 		// timestamp and publish controls
	// 		_act_controls.timestamp = hrt_absolute_time();
	// 		_actuator_controls_pub.publish(_act_controls);

	// 		vehicle_thrust_setpoint_s v_thrust_sp{};
	// 		v_thrust_sp.timestamp = hrt_absolute_time();
	// 		v_thrust_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_THROTTLE];
	// 		v_thrust_sp.xyz[1] = 0.0f;
	// 		v_thrust_sp.xyz[2] = 0.0f;
	// 		_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

	// 		vehicle_torque_setpoint_s v_torque_sp{};
	// 		v_torque_sp.timestamp = hrt_absolute_time();
	// 		v_torque_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_ROLL];
	// 		v_torque_sp.xyz[1] = _act_controls.control[actuator_controls_s::INDEX_PITCH];
	// 		v_torque_sp.xyz[2] = _act_controls.control[actuator_controls_s::INDEX_YAW];
	// 		_vehicle_torque_setpoint_pub.publish(v_torque_sp);
	// 		// PX4_WARN("PUBLISHING.... %f %f",(double)v_thrust_sp.xyz[0], (double)v_torque_sp.xyz[2]);
	// 	}
	// }
	// =========================================================================================================================
}


Vector2f RoverPositionControl::maix(float linear_velocity, float angular_velocity){


	// convert linear and angular velocities to left and right wheel speed
	Vector2f cmd;

	float right_wheel = linear_velocity + (GND_WHEEL_BASE / 2 ) * angular_velocity;
	float left_wheel = linear_velocity - (GND_WHEEL_BASE / 2 ) * angular_velocity;
	cmd(1) = right_wheel;
	cmd(2) = left_wheel;


	return cmd;
	}


// TODO: Change the size of the
void RoverPositionControl::publish_cmd(Vector4f cmd) {
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


int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	RoverPositionControl *instance = new RoverPositionControl();

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

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `vehicle_thrust_setpoint (only in x) and vehicle_torque_setpoint (only yaw)` messages at IMU_GYRO_RATEMAX.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}


extern "C" __EXPORT int rover_control_main(int argc, char *argv[]) {
  return RoverControl::main(argc, argv);
}

/* TODO: Controller methods to implement
_controller.update_state_pos(_state_pos)
_controller.update_state_Omega(_state_ang_vel)
_controller.update_setpoint(_setpoint);
_controller.update_state_attitude(_state_att);
_controller.run();
_controller.get_thrust_cmd()
_controller.get_torque_cmd()

*/
