uint64 timestamp				# time since system start (microseconds)
float32[4] estimate				# First-order actuator estimate before LowPassFilter2p (rad)
float32[4] delta				# LowPassFilter2p output fed back to INDI (rad)
