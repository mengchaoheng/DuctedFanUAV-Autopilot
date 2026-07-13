uint64 timestamp
uint64 timestamp_sample

bool control_flag
bool enabled
bool supported
bool allocation_valid
bool waiting
uint8 u_dim
int32 ca_airframe

float32 mass
float32 force_scale    # Body Fz row of D, [1/N]

float32[3] acc_meas       # NED acceleration, [m/s^2]
float32[3] force_feedback # Filtered allocated force in NED, [N]
