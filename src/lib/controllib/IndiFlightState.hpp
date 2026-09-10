#pragma once

/** Shared INDI flight eligibility for acceleration and rate control.
 * After the first ground contact following flight, keep PID selected until
 * disarm. Ground-contact flicker must not reactivate airborne feedback on the ground.
 */
class IndiFlightState
{
public:
	bool update(bool armed, bool has_taken_off, bool ground_contact, bool maybe_landed, bool landed)
	{
		if (!armed) {
			_airborne_seen = false;
			_touchdown = false;
			return false;
		}

		if (!has_taken_off) {
			return false;
		}

		if (ground_contact || maybe_landed || landed) {
			_touchdown |= _airborne_seen;
			return false;
		}

		_airborne_seen = true;
		return !_touchdown;
	}

private:
	bool _airborne_seen{false};
	bool _touchdown{false};
};
