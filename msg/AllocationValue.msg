uint64 timestamp
uint64 timestamp_sample # timestamp of the torque setpoint that triggered allocation

float32[3] raw_allocated_torque # Unfiltered body-FRD torque from final actuator setpoints [N m]
float32[3] raw_allocated_force  # Unfiltered body-FRD force from final actuator setpoints [N]
float32[3] allocated_torque     # Filtered body-FRD torque from torque matrix final setpoint [N m]
float32[3] allocated_force      # Filtered body-FRD force from force matrix final setpoint [N]
float32[3] torque_setpoint_scale # D_torque: physical torque to normalized torque setpoint
float32[3] force_setpoint_scale  # D_force: physical force to normalized thrust setpoint
