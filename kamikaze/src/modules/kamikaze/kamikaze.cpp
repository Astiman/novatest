#include "kamikaze.hpp"

NovaKamikaze::NovaKamikaze() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "nova_kamikaze"))
{
}

void
NovaKamikaze::update_params(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		updateParams();
		_p_flight_state = _param_flight_state.get();
		_p_kamikaze_dive_alt = _param_kamikaze_dive_alt.get();
		_p_kamikaze_rise_alt = _param_kamikaze_rise_alt.get();
		_p_kamikaze_dive_ang = _param_kamikaze_dive_ang.get();
		_p_kamikaze_rise_ang = _param_kamikaze_rise_ang.get();
		_p_kamikaze_dive_thr = _param_kamikaze_dive_thr.get();
		_p_kamikaze_level_thr = _param_kamikaze_level_thr.get();
		_p_kamikaze_rise_thr = _param_kamikaze_rise_thr.get();
	}
}

void
NovaKamikaze::publish_offboard_control_mode()
{
        _offboard_control.timestamp = hrt_absolute_time();
        _offboard_control.attitude = true;
        _offboard_control.thrust_and_torque = true;
        _offboard_control_pub.publish(_offboard_control);
}

void
NovaKamikaze::switch_mode(int _mode)
{
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1;
	if (_mode = _offboard)
	{
		cmd.param2 = 6; // OFFBOARD ID
	}
	if (_mode = _stabilize)
	{
		cmd.param2 = 7; // OFFBOARD ID
	}

        cmd.target_system = _vehicle_status.system_id;
        cmd.target_component = _vehicle_status.component_id;

        cmd.source_system = _vehicle_status.system_id;
        cmd.source_component = _vehicle_status.component_id;

        cmd.confirmation = false;
        cmd.from_external = false;

        _vehicle_command_pub.publish(cmd);
}

void
NovaKamikaze::set_mode_altitude()
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

void
NovaKamikaze::populate_att_sp(int status)
{
        _position_sub.update(&_position);

	int _roll_ang = 0;
	float _yaw_ang = _position.heading;
	float _pitch_ang;

	if (status == _dive)
	{
		// Euler angles
		_pitch_ang = math::radians(_p_kamikaze_dive_ang);

		//Thrust
		_thrust = _p_kamikaze_dive_thr;
	}
	else if (status == _rise)
	{
		// Euler angles
		_pitch_ang = math::radians(_p_kamikaze_rise_ang);

		//Thrust
		_thrust = _p_kamikaze_rise_thr;
	}
	else
	{
		// Euler angles
		_pitch_ang = 0;

		//Thrust
		_thrust = _p_kamikaze_level_thr;
	}

	matrix::Eulerf euler_angles(_roll_ang, _pitch_ang, _yaw_ang);
	matrix::Quatf quaternion(euler_angles);

	// Populate Setpoint
	_attitude_setpoint = {};

	_attitude_setpoint.timestamp = hrt_absolute_time();

	_attitude_setpoint.q_d[0] = quaternion(0);
	_attitude_setpoint.q_d[1] = quaternion(1);
	_attitude_setpoint.q_d[2] = quaternion(2);
	_attitude_setpoint.q_d[3] = quaternion(3);

	_attitude_setpoint.thrust_body[0] = _thrust;
}

bool
NovaKamikaze::check_alt(int status)
{
        _position_sub.update(&_position);

	if (status == _dive)
	{
		if (-_position.z <= _p_kamikaze_dive_alt * 1.1f)
		{
			return true;
		}
		else
		{
			return false;
		}

	}
	else if (status == _rise)
	{
		if (-_position.z >= _p_kamikaze_rise_alt * 0.9f)
		{
			return true;
		}
		else
		{
			return false;
		}

	}
	else
	{
		return false;
	}

}

bool
NovaKamikaze::check_level()
{
	_attitude_sub.update(&_attitude);

	matrix::Quatf q(_attitude.q);
	matrix::Eulerf euler_angles(q);

	float _pitch_ang = math::degrees(euler_angles.theta());

	if (_pitch_ang >= -10)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int
NovaKamikaze::print_status()
{
	PX4_INFO("Kamikaze dive & rise running");
	return 0;
}

int
NovaKamikaze::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int
NovaKamikaze::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("kamikaze",
					SCHED_DEFAULT,
					SCHED_PRIORITY_HIGH,
					2048,
					(px4_main_t)&run_trampoline,
					(char *const *)argv);
	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

NovaKamikaze *NovaKamikaze::instantiate(int argc, char *argv[])
{
	NovaKamikaze *instance = new NovaKamikaze();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	return instance;
}

void
NovaKamikaze::run()
{
	update_params(true);
	const hrt_abstime loop_period = 1000000 / _loop_rate_hz;

	while (!should_exit())
	{
		if (_parameter_update_sub.updated()) {
                        update_params(false);
                }

		_vehicle_status_sub.update(&_vehicle_status);

		if (_p_flight_state == 1 && !_kamikaze_finished)
		{
			populate_att_sp(_level);
			publish_offboard_control_mode();

			if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD && !_offboard_switch_commanded)
			{
				PX4_INFO("Switching to OFFBOARD");
				set_mode_offboard();
				_offboard_switch_commanded = true;
			}
			while (!should_exit() && vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
			{
				_vehicle_status_sub.update(&_vehicle_status);
				if (!_kamikaze_dive_finished)
				{
					populate_att_sp(_dive);
					_attitude_setpoint_pub.publish(_attitude_setpoint);
					if (!check_alt(_dive))
					{
						_attitude_setpoint_pub.publish(_attitude_setpoint);
						px4_usleep(loop_period);
						continue;
					}
					_kamikaze_dive_finished = true;
				}
				if (!_kamikaze_level_finished)
				{
					populate_att_sp(_level);
					_attitude_setpoint_pub.publish(_attitude_setpoint);
					if (!check_level())
					{
						_attitude_setpoint_pub.publish(_attitude_setpoint);
						px4_usleep(loop_period);
						continue;
					}
					_kamikaze_level_finished = true;
				}
				if (!_kamikaze_rise_finished)
				{
					populate_att_sp(_rise);
					_attitude_setpoint_pub.publish(_attitude_setpoint);
					if (!check_alt(_rise))
					{
						_attitude_setpoint_pub.publish(_attitude_setpoint);
						px4_usleep(loop_period);
						continue;
					}
					_kamikaze_rise_finished = true;
				}

				_kamikaze_dive_finished = false;
				_kamikaze_level_finished = false;
				_kamikaze_rise_finished = false;
				_offboard_switch_commanded = false;


			}
			_kamikaze_finished = true;
			_offboard_switch_commanded = false;
			px4_usleep(loop_period);
			continue;
		}
		else
		{
			px4_usleep(loop_period);
			continue;
		}
	}

	exit_and_cleanup();
	return;
}


int NovaKamikaze::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Kamikaze dive & rise module
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("kamikaze", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int kamikaze_main(int argc, char *argv[])
{
	return NovaKamikaze::main(argc, argv);
}
