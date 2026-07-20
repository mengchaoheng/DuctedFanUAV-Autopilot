uint64 timestamp		# time since system start (microseconds)

bool indi_active		# true when the rate INDI controller produced this cycle's torque setpoint

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
