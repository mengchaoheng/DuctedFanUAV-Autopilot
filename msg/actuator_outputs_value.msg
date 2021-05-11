uint64 timestamp				# time since system start (microseconds)
uint8 NUM_ACTUATOR_OUTPUTS		= 5
uint8 NUM_ACTUATOR_OUTPUT_GROUPS	= 4	# for sanity checking
uint32 noutputs				# valid outputs
float32[5] output				# output data, in natural output units. 0 for thrust, and 1-4 for control surfaces
float32 d_output			# Differential of outputthust
float32[5] min_value
float32[5] max_value
