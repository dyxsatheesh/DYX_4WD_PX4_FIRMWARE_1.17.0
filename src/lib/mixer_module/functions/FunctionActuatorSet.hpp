/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FunctionProviderBase.hpp"
#include "DyxSprayPulse.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>

/**
 * Functions: Peripheral_via_Actuator_Set1 ... Peripheral_via_Actuator_Set6
 */
class FunctionActuatorSet : public FunctionProviderBase
{
public:
	FunctionActuatorSet()
	{
		for (int i = 0; i < max_num_actuators; ++i) {
			_data[i] = NAN;
		}

		// Only the DYX production airframe opts into the spray safety
		// contract; all ordinary PX4 actuator-set functions remain unchanged.
		const param_t autostart = param_find("SYS_AUTOSTART");
		int32_t airframe = 0;

		if (autostart != PARAM_INVALID && param_get(autostart, &airframe) == 0) {
			_dyx_spray_enabled = (airframe == 50002);
		}

		if (_dyx_spray_enabled) {
			_data[0] = DyxSprayPulse::OFF;
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionActuatorSet(); }

	void update() override
	{
		const hrt_abstime now = hrt_absolute_time();
		vehicle_command_s vehicle_command{};
		bool got_spray_command = false;
		bool request_spray_on = false;

		while (_topic.update(&vehicle_command)) {
			if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR
			    || !PX4_ISFINITE(vehicle_command.param7)) {
				continue;
			}

			const int index = (int)(vehicle_command.param7 + 0.5f);

			if (index == 0) {
				if (PX4_ISFINITE(vehicle_command.param1)) {
					if (!_dyx_spray_enabled) {
						_data[0] = vehicle_command.param1;
					} else if (vehicle_command.timestamp > 0 && now >= vehicle_command.timestamp
						   && (now - vehicle_command.timestamp) <= 250'000ULL) {
						// Only a fresh explicit OFF followed by ON can start a pulse.
						// There is no implicit retrigger when the client reconnects.
						got_spray_command = true;
						request_spray_on = vehicle_command.param1 >= 0.9f;
					}
				}

				if (PX4_ISFINITE(vehicle_command.param2)) {_data[1] = vehicle_command.param2; }
				if (PX4_ISFINITE(vehicle_command.param3)) {_data[2] = vehicle_command.param3; }
				if (PX4_ISFINITE(vehicle_command.param4)) {_data[3] = vehicle_command.param4; }
				if (PX4_ISFINITE(vehicle_command.param5)) {_data[4] = vehicle_command.param5; }
				if (PX4_ISFINITE(vehicle_command.param6)) {_data[5] = vehicle_command.param6; }
			}
		}

		if (_dyx_spray_enabled) {
			vehicle_control_mode_s vehicle_control_mode{};
			offboard_control_mode_s offboard_control_mode{};
			trajectory_setpoint_s trajectory_setpoint{};

			const bool control_fresh = _vehicle_control_mode_sub.copy(&vehicle_control_mode)
				&& vehicle_control_mode.timestamp > 0 && now >= vehicle_control_mode.timestamp
				&& (now - vehicle_control_mode.timestamp) < 500'000ULL;
			const bool heartbeat_fresh = _offboard_control_mode_sub.copy(&offboard_control_mode)
				&& offboard_control_mode.timestamp > 0 && now >= offboard_control_mode.timestamp
				&& (now - offboard_control_mode.timestamp) < 500'000ULL;
			const bool setpoint_fresh = _trajectory_setpoint_sub.copy(&trajectory_setpoint)
				&& trajectory_setpoint.timestamp > 0 && now >= trajectory_setpoint.timestamp
				&& (now - trajectory_setpoint.timestamp) < 500'000ULL;
			const bool permitted = control_fresh && heartbeat_fresh && setpoint_fresh
				&& vehicle_control_mode.flag_armed && vehicle_control_mode.flag_control_offboard_enabled
				&& offboard_control_mode.velocity && offboard_control_mode.body_rate
				&& !offboard_control_mode.position && !offboard_control_mode.attitude;

			_data[0] = _spray_pulse.update(now, permitted, got_spray_command, request_spray_on);
		}
	}

	float value(OutputFunction func) override { return _data[(int)func - (int)OutputFunction::Peripheral_via_Actuator_Set1]; }

	float defaultFailsafeValue(OutputFunction func) const override
	{
		return (_dyx_spray_enabled && func == OutputFunction::Peripheral_via_Actuator_Set1)
		       ? DyxSprayPulse::OFF : NAN;
	}

private:
	static constexpr int max_num_actuators = 6;

	uORB::Subscription _topic{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	DyxSprayPulse _spray_pulse{};
	bool _dyx_spray_enabled{false};
	float _data[max_num_actuators] {};
};
