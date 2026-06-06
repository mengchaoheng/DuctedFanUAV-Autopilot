
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)

# Optional same-cycle split of xyz for prioritized allocation.
# When xyz_split_valid is true:
#   xyz = xyz_rate_error_feedback + xyz_indi_feedback
float32[3] xyz_rate_error_feedback # lower-priority torque component (normalized)
float32[3] xyz_indi_feedback       # higher-priority INDI torque component (normalized)
bool xyz_split_valid

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
