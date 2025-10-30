#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/offboard_control_mode.h>

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

using namespace time_literals;

class KamikazeDive : public ModuleBase<KamikazeDive>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	KamikazeDive();
	~KamikazeDive() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void populate_att_sp();
	void publish_offboard_control_mode();
        void set_mode_offboard();

	// Publications
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<offboard_control_mode_s>	_offboard_control_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> 		_vehicle_command_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription                 _position_sub{ORB_ID(vehicle_local_position)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Structs
	vehicle_status_s    		_vehicle_status{};
	vehicle_attitude_setpoint_s	_attitude_setpoint{};
	offboard_control_mode_s		_offboard_control{};
	vehicle_local_position_s		_position{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FLIGHT_STATE>) _param_flight_state,
		(ParamInt<px4::params::KAMIK_DIVE_ALT>) _param_dive_alt,
		(ParamInt<px4::params::KAMIK_DIVE_ANG>) _param_dive_ang,
		(ParamFloat<px4::params::KAMIK_DIVE_THR>) _param_dive_thr
	)
};
