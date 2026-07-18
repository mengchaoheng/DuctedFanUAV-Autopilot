
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)

# Optional higher-priority INDI component for PCA.
float32[3] xyz_indi_feedback # higher-priority INDI torque component (normalized)
bool xyz_indi_feedback_valid # true when this setpoint contains an INDI priority split

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
