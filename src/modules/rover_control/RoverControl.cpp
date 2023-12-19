#include "RoverControl.hpp"

using namespace matrix;

// ===================== Old code ===========================
// README: In the new version I removed the performace counter and added the paramters_update; Following Dev's structure

// RoverControl::RoverControl() :
// 	ModuleParams(nullptr),
// 	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)


// 	/* performance counters */
// 	// Removing this for now
// 	_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle")) // TODO : do we even need these perf counters
// {
// }
// ==========================================================


RoverControl::RoverControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {

  // initializer
  parameters_update();
}

RoverControl::~RoverControl()
{
	perf_free(_loop_perf);
}

bool
RoverControl::init()
{
	if (!_ang_vel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_timestamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void RoverControl::parameters_update()
{
	// check for parameter updates
  if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		PX4_WARN("updateParams"); // Add warning that params are updated

		set_wheel_base(_param_wheel_base.get());
		set_controller_gains(_param_rover_kx.get(), _param_rover_kv.get(), _param_rover_komega.get());

		_rover_speed_max = _param_speed_max.get();
	}
}

void
RoverControl::Run()
{
	if (should_exit()) {
	_ang_vel_sub.unregisterCallback();
	exit_and_cleanup();
	return;
	}

	perf_begin(_cycle_perf);

	// Only progress if ang velocity is updated; otherwise, return
	if (!_ang_vel_sub.updated()) {
	perf_end(_cycle_perf);
	return;
	}

	parameters_update(); // TODO: add mixer parameters and controller parameters

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	// To only test raw commands
	// publish_cmd(cmd_);
	// perf_end(_cycle_perf);
	// return;

	// grab commander status
	if (_commander_status_sub.update(&_commander_status)) {
		_init_commander_status = true;
		_armed = _commander_status.state != commander_status_s::STATE_DISARMED;
	}

	// If the commander status is disarmed then velocity = 0.0 is published
	if (!_armed) {
		float v = 0.0;
		Vector2f cmd(v, v);
		publish_cmd(cmd);
		perf_end(_cycle_perf);
		return;
	}

	// update other subscribers
	if (_ang_vel_sub.update(&angular_velocity)) {
		rover_omega(0) = angular_velocity.xyz[0];
		rover_omega(1) = angular_velocity.xyz[1];
		rover_omega(2) = angular_velocity.xyz[2];
		_init_state_omega = true;
	}

	if (_local_pos_sub.update(&_state_pos)) {

		update_state_pos(_state_pos);

		// if the mode is not landing, update the landing state
		if (_init_commander_status && _commander_status.state != commander_status_s::STATE_LAND) {
			_local_pos_sub.copy(&_start_landing_state);
			_last_timestamp_land_started = hrt_absolute_time();
		}
		_init_state_pos = true;
	}

	if (_trajectory_setpoint_sub.update(&_setpoint)) {
		update_setpoint(_setpoint);
		_init_setpoint = true;
	}

	if (_att_sub.update(&_state_att)) {
		update_yaw(_state_att);
	 	_init_state_att = true;
	}

	_initialized = _init_state_omega && _init_state_pos && _init_state_att &&
			_init_setpoint && _init_commander_status;

	// === to test the motors ============
	// float v = 0.0;

	// Vector2f cmd(-1.0f, -1.0f);
	// publish_cmd(cmd_);
	// ====================================
	// PX4_INFO("commander status: %d, arming status: %d, _init: %d", _commander_status.state, _armed, _initialized);

	if (!_initialized) {
		float v = 0.05;
		Vector2f cmd(v, v); // TODO: Need to change this
		publish_cmd(cmd);
		PX4_WARN("ARMED but not INITIALIZED");
		perf_end(_cycle_perf);
		return;
	}

	// if in armed mode, publish the min PWM
	// PX4_WARN("_commander_status: %d ", _commander_status.state);
	// if (_commander_status.state == commander_status_s::STATE_ARMED) {
	// 	float v = 0.0;
	// 	Vector2f cmd(v, v);
	// 	publish_cmd(cmd);
	// 	perf_end(_cycle_perf);
	// 	return;
	// }

	// ================================== LAND =====================================
	if (_commander_status.state == commander_status_s::STATE_LAND) {

		// Stay there!
		// Set _setpoint raw_mode to true and give the raw commands of zero angular and linear velocity
		_setpoint.raw_mode = true;
		_setpoint.cmd[0] = 0.0;
		_setpoint.cmd[1] = 0.0;
	}
	// ==============================================================================

	// if the code is here, it is in OFFBOARD MODE and with the correct setpoint

	PX4_WARN("Going into OFFBOARD MODE");

	if (_setpoint.raw_mode == false) {
		PX4_WARN("raw mode was false");
		// run controller
		// _controller.run();

		Vector2f linear_angular_cmd = rover_controller().zero_if_nan();

		// do the mixing: Giving linear and angular velocity as arguments
		Vector2f cmd = mix(linear_angular_cmd(0), linear_angular_cmd(1));
		// cmd.print();
		// publish
		publish_cmd(cmd);

	} else {
		// *** What is the RAW MOTOR Mode // In rover's case should be the direct right and left wheel
		PX4_WARN("IN RAW MOTOR MODE");

		// copy from the _setpoint msg
		Vector2f motor_cmd(_setpoint.cmd[0], _setpoint.cmd[1]);

		// publish
		publish_cmd(motor_cmd);
	}

	perf_end(_cycle_perf);

	return;

}

// This is done tp avoid param_wheel_base.get() which might be computationally expensive
void RoverControl::set_wheel_base(float b) { _rover_wheel_base = b; }
void RoverControl::set_controller_gains(float position_gain, float velocity_gain, float angle_gain){
	kx = position_gain;
	kv = velocity_gain;
	komega = angle_gain;
}

Vector2f RoverControl::mix(float linear_velocity, float angular_velocity){

	// convert linear and angular velocities to left and right wheel speed
	Vector2f cmd;
	// float right_wheel_uncon =  linear_velocity + (_rover_wheel_base / 2 ) * angular_velocity;
	float right_wheel = math::constrain( linear_velocity - (_rover_wheel_base / 2.0f ) * angular_velocity, -_rover_speed_max, _rover_speed_max);
	float left_wheel  = math::constrain( linear_velocity + (_rover_wheel_base / 2.0f ) * angular_velocity, -_rover_speed_max, _rover_speed_max);

	// PX4_INFO("lin: %f, ang: %f, right: %f, right_con: %f", (double)linear_velocity, (double)angular_velocity, (double)right_wheel_uncon, (double)right_wheel);

	cmd(0) = right_wheel * 1.0f / _rover_speed_max;
	cmd(1) = left_wheel * 1.0f / _rover_speed_max;

	// cmd(0) = angular_velocity; //(_rover_wheel_base / 2.0f ) * angular_velocity;
	// cmd(1) = -angular_velocity; //- (_rover_wheel_base / 2.0f ) * angular_velocity;

	return cmd;
}

void RoverControl::update_state_pos(vehicle_local_position_s pos){
	rover_pos(0) = pos.x;
	rover_pos(1) = pos.y;
	rover_pos(2) = 0.0f;//pos.z;

	rover_vel(0) = pos.vx;
	rover_vel(1) = pos.vy;
	rover_vel(2) = 0.0;//pos.vx;
}

void RoverControl::update_yaw(vehicle_attitude_s att) {
  // grab rotation from body frame to earth frame
  rover_yaw = Eulerf(Quatf(att.q)).psi();

}

void RoverControl::update_setpoint(trajectory_setpoint_s sp){

	for (size_t i = 0; i < 2; i++) {
		pos_ref(i) = sp.position[i];
	}
	pos_ref(2) = 0.0;

	yaw_ref = -sp.yaw;
	angular_vel_ref = -sp.yawspeed;
	linear_vel_ref = sp.velocity[0];
}

float RoverControl::wrap_angle(float angle){
	// return std::atan2( sinf(angle), cosf(angle) );
	if ( angle > M_PI_F ){
		return wrap_angle( angle - 2.0f * M_PI_F );
	}
	else if ( angle < -M_PI_F ){
		return wrap_angle( angle + 2.0f * M_PI_F );
	}
	else{
		return angle;
	}
}

Vector2f RoverControl::rover_controller(){



	Vector3f  ex = (pos_ref - rover_pos).zero_if_nan();
	yaw_ref = std::atan2( ex(1), ex(0) );
	// yaw_ref = 0.0f;

	float yaw_error = wrap_angle(yaw_ref - rover_yaw);

	// yaw_error = 3.0f;
	// ex(0) = 0.0f;
	// ex(1) = 0.0f;
	// ex(2) = 0.0f;
	// // PX4_INFO("ex %f, %f, %f", (double)ex(0), (double)ex(1), (double)ex(2));
	// ex.print();
	// rover_pos.print();


	const Dcmf R(matrix::Eulerf(0.0f,0.0f,rover_yaw));// body to world frame
	// // R(0,0) =  cos(rover_yaw); R(0,1) = -sin(rover_yaw); R(0,2) = 0.0;
	// // R(1,0) =  sin(rover_yaw); R(1,1) =  cos(rover_yaw); R(1,2) = 0.0;
	// // R(2,0) =  0.0           ; R(2,1) = 0.0            ; R(2,2) = 1.0;

	Vector3f rover_vel_body = R.T() * rover_vel;
	float x_vel = rover_vel_body(0);

	float linear_vel_cmd = 0;
	float angular_vel_cmd = 0;

	if (1){//( ex.norm() > _rover_wheel_base/2 ){
		float desired_linear_vel = kx * ex.norm() * cosf(yaw_error);
		linear_vel_cmd = desired_linear_vel + kv * ( desired_linear_vel - x_vel );
		angular_vel_cmd = komega * yaw_error;
	}

	Vector2f lin_ang_cmd;
	lin_ang_cmd(0) = linear_vel_cmd;
	lin_ang_cmd(1) = angular_vel_cmd;
	// lin_ang_cmd(0) = 0.0f;
	// lin_ang_cmd(1) = 2.0f;
	// lin_ang_cmd.print();

	return lin_ang_cmd;

	// float k1 = 1.0f;
	// float k2 = 1.0f;
	// float k3 = 1.0f;


	// Vector2f lin_ang_cmd;
	// // Error coordinates
	// float xe = cosf(yaw_ref) * (rover_pos(0) - pos_ref(0)) + sinf(yaw_ref) * (rover_pos(1) - pos_ref(1));
	// float ye = -sinf(yaw_ref) * (rover_pos(0) - pos_ref(0)) + cosf(yaw_ref) * (rover_pos(1) - pos_ref(1));


	// float linear_vel_cmd = (linear_vel_ref - k1 * fabsf(linear_vel_ref) * (xe + ye * tanf(yaw_error)))/cosf(yaw_error);
	// float angular_vel_cmd = angular_vel_ref - (k2 * linear_vel_ref * ye + k3 * fabsf(linear_vel_ref) * tanf(yaw_error)) * powf(cosf(yaw_error), 2);

	// lin_ang_cmd(0) = linear_vel_cmd;
	// lin_ang_cmd(1) = angular_vel_cmd;

	// return lin_ang_cmd;

}



// ================================================================

void RoverControl::handle_command_raw(Vector2f cmd)
{
	cmd_ = cmd;
}



void RoverControl::handle_command_pos(Vector2f pos)
{
	// Set everything to zero
	for (size_t i = 0; i < 3; i++) {
	_setpoint.position[i] = 0.0f;
	_setpoint.velocity[i] = 0.0f;
	_setpoint.acceleration[i] = 0.0f;
	_setpoint.jerk[i] = 0.0f;
	}

	// Set x and y to pos command
	_setpoint.position[0] = pos(0);
	_setpoint.position[1] = pos(1);

	PX4_INFO("Updated Setpoint");
	update_setpoint(_setpoint);
	_init_setpoint = true;
}

void RoverControl::publish_cmd(Vector2f cmd) {
  // cmd(0) -> right wheel speed
  // cmd(1) -> left wheel speed

//   PX4_INFO("publishing cmd: %f, %f", (double)cmd(0), (double)cmd(1));
  {
    actuator_outputs_s msg;
    msg.timestamp = hrt_absolute_time();
    msg.noutputs = 2;
    for (size_t i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
      msg.output[i] = 0.0f;
    }
    msg.output[0] = cmd(0);
    msg.output[1] = cmd(1);
    _actuator_outputs_pub.publish(msg);
    _actuator_outputs_sim_pub.publish(msg);
  }

  {
    // publish for hardware
    actuator_motors_s msg;
    msg.timestamp = hrt_absolute_time();
    msg.reversible_flags = 3; // all motors are reversible
    // set all to 0 first
    for (size_t i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
      msg.control[i] = 0;
    }
    // set the desired ones to the desired values
    for (size_t i = 0; i < 2; i++) {
      msg.control[i] = cmd(i);
    }
    PX4_INFO("Publishing to motors pub");
    _actuator_motors_pub.publish(msg);
  }
}


int RoverControl::task_spawn(int argc, char *argv[])
{
	RoverControl *instance = new RoverControl();

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

int RoverControl::custom_command(int argc, char *argv[]){

  if (!strcmp(argv[0], "raw")) {
	Vector2f cmd  = {(float)atof(argv[1]), (float)atof(argv[2]) };
	PX4_INFO("SENDING RAW COMMAND %.2f %.2f", (double)cmd(0), (double)cmd(1));

    get_instance()->handle_command_raw(cmd);
    return 0;
  }

  if (!strcmp(argv[0], "pos")) {

    Vector2f pos  = {(float)atof(argv[1]), (float)atof(argv[2]) };
    get_instance()->handle_command_pos(pos);
    return 0;
  }


	return print_usage("unknown command");
}

int RoverControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
			### Description
			Controls the position of a ground rover using custom DASC controller
			### Examples
			CLI usage example:
			$ rover_control start
			$ rover_control status
			$ rover_control stop
			$ rover_control raw 0.5 0.5 # set left and right speeds

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_COMMAND("raw")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rover_control_main(int argc, char *argv[]) {
  return RoverControl::main(argc, argv);
}
