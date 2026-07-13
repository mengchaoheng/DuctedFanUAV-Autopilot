uint64 timestamp
uint64 timestamp_sample

bool control_flag
bool enabled
bool allocation_valid
uint8 u_dim
float32 indi_dt

float32[3] error_fb                 # J*Kp*(rate_sp-rate), [N m]
float32[3] indi_fb                  # tau_0-J*angular_accel, [N m]
float32[3] output                   # Commanded physical moment, [N m]
float32[3] output_normalized        # D*output, dimensionless
float32[3] control_allocation_scale # Torque rows of D, [1/(N m)]
