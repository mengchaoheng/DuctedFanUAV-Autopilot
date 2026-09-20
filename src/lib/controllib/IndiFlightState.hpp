#pragma once

/** Shared INDI flight eligibility for acceleration and rate control.
 * Ground detection after flight requires a new parameter or RC off-to-on edge.
 * Disarming clears the latch for the next flight. Ground operation is always PID.
 */
class IndiFlightState
{
public:
	bool update(bool armed, bool has_taken_off, bool ground_contact, bool maybe_landed, bool landed,
		    bool parameter_enabled, bool rc_enabled, bool rc_valid)
	{
		const bool on_ground = ground_contact || maybe_landed || landed;
		const bool airborne = armed && has_taken_off && !on_ground;
		const bool retrigger = (parameter_enabled && !_parameter_enabled_previous)
				       || (rc_valid && rc_enabled && !_rc_enabled_previous);

		if (!armed) {
			_retrigger_required = false;

		} else if (_airborne_previous && on_ground) {
			_retrigger_required = true;

		} else if (retrigger) {
			_retrigger_required = false;
		}

		_airborne_previous = airborne;
		_parameter_enabled_previous = parameter_enabled;

		// Signal loss/recovery must not turn a held-high RC switch into an edge.
		if (rc_valid) {
			_rc_enabled_previous = rc_enabled;
		}

		// Request gating remains in the caller, preserving normal PID/INDI blending.
		return airborne && !_retrigger_required;
	}

private:
	bool _airborne_previous{false};
	bool _parameter_enabled_previous{false};
	bool _rc_enabled_previous{false};
	bool _retrigger_required{false};
};
