uint64 timestamp		# time since system start (microseconds)
bool valid			# true when thrust_u_ultimate is valid for acceleration INDI
uint8 output_index		# output channel used by the first thrust motor, currently 0 for df4/df6/SHC09/SHW09
float32 thrust_u		# first thrust motor command in normalized motor units [0, 1]
float32 thrust_u_ultimate	# filtered first thrust motor feedback in normalized motor units [0, 1]
float32 thrust_umin		# first thrust motor lower limit in normalized motor units
float32 thrust_umax		# first thrust motor upper limit in normalized motor units
