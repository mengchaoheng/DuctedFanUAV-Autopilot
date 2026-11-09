uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample # gyro sample time [us] for MC rate timing, 0 if unavailable

uint32 compute_us       # selected MC rate controller: prepared feedback to normalized torque [us] when timestamp_sample is nonzero

bool rate_indi_active		# true when the rate INDI controller produced this cycle's torque setpoint

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
