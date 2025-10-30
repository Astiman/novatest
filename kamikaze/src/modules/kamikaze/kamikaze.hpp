#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>


class NovaKamikaze : public ModuleBase<NovaKamikaze>
{
public:
	NovaKamikaze();
	virtual ~NovaKamikaze() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static NovaKamikaze *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	// Functions
	void update_params(bool force = false);
	void populate_att_sp(int status);

	void publish_offboard_control_mode();
        void set_mode_offboard();

	//Variables
	float _thrust;

	int _p_flight_state;
	int _p_kamikaze_dive_alt;
	int _p_kamikaze_dive_ang;
	float _p_kamikaze_dive_thr;

	bool _offboard_switch_commanded;
	bool _kamikaze_dive_finished{false};
	bool _kamikaze_level_finished{false};
	bool _kamikaze_rise_finished{false};
	bool _kamikaze_finished{false};

	int _loop_rate_hz{100};


	//Subscriptions
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	//Publications
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<offboard_control_mode_s>	_offboard_control_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> 		_vehicle_command_pub{ORB_ID(vehicle_command)};

	//Structs
	vehicle_status_s    		_vehicle_status{};
	vehicle_attitude_s		_attitude{};
	vehicle_attitude_setpoint_s	_attitude_setpoint{};
	vehicle_local_position_s	_position{};
	offboard_control_mode_s		_offboard_control{};

	perf_counter_t 		_loop_perf;
	hrt_abstime 		_last_run_time{0};

	//Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FLIGHT_STATE>) _param_flight_state,
		(ParamInt<px4::params::KAMIK_DIVE_ALT>) _param_kamikaze_dive_alt,
		(ParamInt<px4::params::KAMIK_RISE_ALT>) _param_kamikaze_rise_alt,
		(ParamInt<px4::params::KAMIK_DIVE_ANG>) _param_kamikaze_dive_ang,
		(ParamInt<px4::params::KAMIK_RISE_ANG>) _param_kamikaze_rise_ang,
		(ParamFloat<px4::params::KAMIK_DIVE_THR>) _param_kamikaze_dive_thr,
		(ParamFloat<px4::params::KAMIK_LEVEL_THR>) _param_kamikaze_level_thr,
		(ParamFloat<px4::params::KAMIK_RISE_THR>) _param_kamikaze_rise_thr
	)
}
