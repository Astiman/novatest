#include "kamikaze_dive.hpp"

KamikazeDive::KamikazeDive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

KamikazeDive::~KamikazeDive()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


void KamikazeDive::publish_offboard_control_mode()
{
        _offboard_control.timestamp = hrt_absolute_time();
        _offboard_control.attitude = true;
        _offboard_control.thrust_and_torque = true;
        _offboard_control_pub.publish(_offboard_control);
}

void KamikazeDive::set_mode_offboard()
{
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1;
	cmd.param2 = 6; // OFFBOARD ID

        cmd.target_system = _vehicle_status.system_id;
        cmd.target_component = _vehicle_status.component_id;

        cmd.source_system = _vehicle_status.system_id;
        cmd.source_component = _vehicle_status.component_id;

        cmd.confirmation = false;
        cmd.from_external = false;

        _vehicle_command_pub.publish(cmd);
}


void KamikazeDive::populate_att_sp()
{
        _position_sub.update(&_position);

	int roll_ang = 0;
	float yaw_ang = _position.heading;
	float pitch_ang = math::radians(_param_dive_ang.get());
	float thrust = _param_dive_thr.get();

	matrix::Eulerf euler_angles(roll_ang, pitch_ang, yaw_ang);
	matrix::Quatf quaternion(euler_angles);

	// Populate Setpoint
	_attitude_setpoint = {};

	_attitude_setpoint.timestamp = hrt_absolute_time();

	_attitude_setpoint.q_d[0] = quaternion(0);
	_attitude_setpoint.q_d[1] = quaternion(1);
	_attitude_setpoint.q_d[2] = quaternion(2);
	_attitude_setpoint.q_d[3] = quaternion(3);

	_attitude_setpoint.thrust_body[0] = thrust;
}


bool KamikazeDive::init()
{
	ScheduleOnInterval(50_ms);

	return true;
}

void KamikazeDive::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	if (_param_flight_state.get() == 1) {

		if(_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			PX4_INFO("Switching to OFFBOARD");
			publish_offboard_control_mode();
			set_mode_offboard();
			return;
		}

		populate_att_sp();
		_attitude_setpoint_pub.publish(_attitude_setpoint);
	}



	perf_end(_loop_perf);
}

int KamikazeDive::task_spawn(int argc, char *argv[])
{
	KamikazeDive *instance = new KamikazeDive();

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

int KamikazeDive::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int KamikazeDive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int KamikazeDive::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
kamikaze dive module
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("kamikaze_dive", "setpoint");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int kamikaze_dive_main(int argc, char *argv[])
{
	return KamikazeDive::main(argc, argv);
}
