/****************************************************************************
 *
 * DYX 4WD rover: deterministic spray pulse state machine.
 * Hardware-independent to allow host unit tests.
 *
 ****************************************************************************/
#pragma once

#include <stdint.h>

class DyxSprayPulse
{
public:
	static constexpr uint64_t PULSE_US = 3'000'000ULL;
	static constexpr float OFF = -1.f; // Configured PWM_AUX_MIN5 == 1200 us
	static constexpr float ON = 1.f;   // Configured PWM_AUX_MAX5 == 1400 us

	// A full OFF->ON transaction is required for every pulse.
	// A loss of the rover control permit cancels any active pulse and
	// disarms the edge detector, even after the DDS connection recovers.
	float update(uint64_t now, bool control_permitted, bool command_received, bool request_on)
	{
		if (!control_permitted) {
			_active = false;
			_armed_by_off = false;
			return OFF;
		}

		if (command_received) {
			if (!request_on) {
				_active = false;
				_armed_by_off = true;

			} else if (_armed_by_off && !_active) {
				_armed_by_off = false;
				_active = true;
				_end_us = now + PULSE_US;
			}
		}

		if (_active && now >= _end_us) {
			_active = false;
		}

		return _active ? ON : OFF;
	}

private:
	bool _armed_by_off{false};
	bool _active{false};
	uint64_t _end_us{0};
};
