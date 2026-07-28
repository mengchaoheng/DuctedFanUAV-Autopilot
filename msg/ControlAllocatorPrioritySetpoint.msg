uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the torque setpoint this message belongs to (microseconds)

float32[3] xyz          # higher-priority torque component about X, Y, Z body axis (normalized)
bool valid              # true when the priority component is valid

# TOPICS control_allocator_priority_setpoint control_allocator_priority_setpoint_virtual_mc
