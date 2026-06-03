uint64 timestamp		# time since system start (microseconds)

uint8 METHOD_RATE_CONTROL = 0
uint8 METHOD_INDI = 1
uint8 method

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ

float32 rate_control_running_time # [us] latest angular rate controller calculation time
float32 rate_control_running_time_avg # [us] online average angular rate controller calculation time
uint32 rate_control_running_time_samples
