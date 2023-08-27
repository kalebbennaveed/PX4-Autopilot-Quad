#include <float.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp> // from Dev
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp> // Dev
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h> // Dev
#include <uORB/topics/actuator_outputs.h> // Dev
#include <uORB/topics/commander_status.h> // Dev
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h> // Dev

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/mathlib/mathlib.h>

// #include <lib/pid/pid.h>
// #include <px4_platform_common/px4_config.h>
// #include <px4_platform_common/tasks.h>
// #include <uORB/SubscriptionInterval.hpp>
// #include <uORB/Publication.hpp>
// #include <uORB/topics/manual_control_setpoint.h>
// #include <uORB/topics/position_controller_status.h>
// #include <uORB/topics/position_setpoint_triplet.h>
// #include <uORB/topics/vehicle_attitude_setpoint.h>
// #include <uORB/topics/vehicle_control_mode.h>
// #include <uORB/topics/vehicle_global_position.h>
// #include <uORB/topics/vehicle_thrust_setpoint.h>
// #include <uORB/topics/vehicle_torque_setpoint.h>

using matrix::Dcmf;
using namespace matrix;

using namespace time_literals;

class RoverControl final : public ModuleBase<RoverControl>,
				   public ModuleParams,
				   public px4::WorkItem {
public:
	RoverControl();
	~RoverControl();
	RoverControl(const RoverControl &) = delete;
	RoverControl operator=(const RoverControl &other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	// void handle_motor_test(int ind);
	// void handle_motor_test_stop();

	// Controller and mix functions
	void set_wheel_base(float b);
	void set_controller_gains(float position_gain, float velocity_gain, float angle_gain, float max_wheel_speed);
	Vector2f mix(float linear_velocity, float angular_velocity);// So mixer receives the linear vel and angular vel and then returns the right and left wheel speed
	void update_state_pos(vehicle_local_position_s pos);
	void update_yaw(vehicle_attitude_s att);
	void update_setpoint(trajectory_setpoint_s sp);
	float wrap_angle(float angle);
	Vector2f Rover_Controller();

private:
	bool init();
	void parameters_update();
	void Run() override;
	void publish_cmd(Vector2f pwm_cmd);


	// Publications
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};
	uORB::Publication<actuator_outputs_s> _actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};

	//Subscriptions
	uORB::Subscription _commander_status_sub{ORB_ID(commander_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _acc_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionCallbackWorkItem _ang_vel_sub{
	this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Private Variables
	vehicle_status_s _vehicle_status;
	vehicle_local_position_s _state_pos;
	vehicle_local_position_s _start_landing_state;
	vehicle_attitude_s _state_att;
	vehicle_acceleration_s _state_acc;
	vehicle_angular_velocity_s _state_ang_vel;
	trajectory_setpoint_s _setpoint;
	commander_status_s _commander_status;

	// from old code
	// manual_control_setpoint_s		_manual_control_setpoint{};			    /**< r/c channel data */

	// Rover state
	Vector3f rover_pos;
	Vector3f rover_vel;
	float rover_yaw;
	Vector3f rover_omega;

	// SETPOINT
	Vector3f pos_ref;
	float yaw_ref, angular_vel_ref, linear_vel_ref;

	// Rover Controller gains
	float kx, kv, komega;
	float _rover_wheel_base;
	float _rover_speed_max;
	float _rover_throttle_min;
	float _rover_throttle_max;
	float _rover_ff_pwm;
	float _rover_wheel_max_speed;

	float _land_speed = 0.2f;

	bool _armed = false;
	bool _initialized = false;
	bool _init_state_omega = false;
	bool _init_state_pos = false;
	bool _init_state_att = false;
	bool _init_state_acc = false;
	bool _init_setpoint = false;
	bool _init_commander_status = false;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	hrt_abstime _timestamp_last_loop{0};
	hrt_abstime _last_timestamp_land_started{0};
	perf_counter_t _cycle_perf{
		perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle time")};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ROVER_SPEED_MAX>) _param_speed_max,
		(ParamFloat<px4::params::ROVER_THR_MIN>) _param_thr_min,
		(ParamFloat<px4::params::ROVER_THR_MAX>) _param_thr_max,
		(ParamFloat<px4::params::GND_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::ROVER_KX>) _param_rover_kx,
		(ParamFloat<px4::params::ROVER_KV>) _param_rover_kv,
		(ParamFloat<px4::params::ROVER_KOMEGA>) _param_rover_komega,
		(ParamFloat<px4::params::ROVER_WHEEL_MAX>) _param_wheel_max_speed)

};
