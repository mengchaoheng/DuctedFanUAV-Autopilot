uint64 timestamp				# time since system start (microseconds)
uint8 NUM_MIXER_OUTPUTS = 16
uint8 ACTUATOR_TYPE_SERVO = 0
uint8 ACTUATOR_TYPE_MOTOR = 1
uint32 noutputs				# valid mixer outputs
uint8[16] actuator_type			# ACTUATOR_TYPE_* for each output slot
float32[16] output				# raw mixer outputs before PWM output_limit_calc()
float32[16] output_command			# command after USER_ACTUATOR virtual-actuator selection
float32[16] output_estimate			# filtered virtual-actuator estimate in mixer-output units
# This topic is intentionally unit-agnostic. For C: ControlAllocationMixer and
# y = B u consumers, use allocation_value.u_ultimate, whose units match B and U.
