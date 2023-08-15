/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

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

class RoverPositionControl final : public ModuleBase<RoverControl>,
				   public ModuleParams,
				   public px4::WorkItem {
public:
	RoverPositionControl();
	~RoverPositionControl();
	RoverPositionControl(const RoverPositionControl &) = delete;
	RoverPositionControl operator=(const RoverPositionControl &other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);



private:
	bool init();
	void parameters_update(bool force = false);
	void Run() override;
	void publish_cmd(Vector4f pwm_cmd);

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
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};

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
	manual_control_setpoint_s		_manual_control_setpoint{};			    /**< r/c channel data */

	// from Dev
	//MixerQuadratic _mixer;. // TODO: make mixer file for rover


	float _land_speed = 0.2f;

	bool _armed = false;
	bool _initialized = false;
	bool _init_state_Omega = false;
	bool _init_state_pos = false;
	bool _init_state_att = false;
	bool _init_state_acc = false;
	bool _init_setpoint = false;
	bool _init_commander_status = false;

	// perf_counter_t	_loop_perf;			/**< loop performance counter */
	hrt_abstime _timestamp_last_loop{0};
	hrt_abstime _last_timestamp_land_started{0};
	perf_counter_t _cycle_perf{
	perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle time")};


	// Controller functions
	void 		control_local_position(const matrix::Vector2f &current_position, const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity);
	void		control_velocity(const matrix::Vector3f &current_velocity, const matrix::Vector3f &current_angular_velocity, const float desired_linear_x_velocity, const float desired_angular_z_velocity);
	void		control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GND_L1_PERIOD>) _param_l1_period,
		(ParamFloat<px4::params::GND_L1_DAMPING>) _param_l1_damping,
		(ParamFloat<px4::params::GND_L1_DIST>) _param_l1_distance,

		(ParamFloat<px4::params::GND_SPEED_TRIM>) _param_gndspeed_trim,
		(ParamFloat<px4::params::GND_SPEED_MAX>) _param_gndspeed_max,

		(ParamInt<px4::params::GND_SP_CTRL_MODE>) _param_speed_control_mode,
		(ParamFloat<px4::params::GND_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::GND_SPEED_I>) _param_speed_i,
		(ParamFloat<px4::params::GND_SPEED_D>) _param_speed_d,
		(ParamFloat<px4::params::GND_SPEED_IMAX>) _param_speed_imax,
		(ParamFloat<px4::params::GND_SPEED_THR_SC>) _param_throttle_speed_scaler,

		(ParamFloat<px4::params::GND_THR_MIN>) _param_throttle_min,
		(ParamFloat<px4::params::GND_THR_MAX>) _param_throttle_max,
		(ParamFloat<px4::params::GND_THR_CRUISE>) _param_throttle_cruise,

		(ParamFloat<px4::params::GND_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::GND_MAX_ANG>) _param_max_turn_angle,
		(ParamFloat<px4::params::GND_MAN_Y_MAX>) _param_gnd_man_y_max,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad	/**< loiter radius for Rover */
	)

	// hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */
	// hrt_abstime _manual_setpoint_last_called{0};

	// MapProjection _global_local_proj_ref{};

	// /* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	//  the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	// PID_t _speed_ctrl{};
	// PID_t _angular_speed_ctrl{};

	// // estimator reset counters
	// uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position

	// ECL_L1_Pos_Controller				_gnd_control;

	// enum UGV_POSCTRL_MODE {
	// 	UGV_POSCTRL_MODE_AUTO,
	// 	UGV_POSCTRL_MODE_OTHER
	// } _control_mode_current{UGV_POSCTRL_MODE_OTHER};			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.


	// enum POS_CTRLSTATES {
	// 	GOTO_WAYPOINT,
	// 	STOPPING
	// } _pos_ctrl_state {STOPPING};			/// Position control state machine

	// /* previous waypoint */
	// matrix::Vector2d _prev_wp{0, 0};

	// enum class VelocityFrame {
	// 	NED,
	// 	BODY,
	// } _velocity_frame{VelocityFrame::NED};

	// float _manual_yaw_sp{0.0};
	// bool _reset_yaw_sp{true};
	// // float _throttle_control{0.f};
	// // float _yaw_control{0.f};





	// void		position_setpoint_triplet_poll();
	// void		attitude_setpoint_poll();
	// void		vehicle_control_mode_poll();
	// void 		vehicle_attitude_poll();
	// void		manual_control_setpoint_poll();


};
