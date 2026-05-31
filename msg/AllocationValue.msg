uint64 timestamp
uint64 timestamp_sample

uint8 MAX_Y = 6
uint8 MAX_U = 16

uint8 y_dim
uint8 u_dim
bool indi_valid
int8 requested_method
uint8 fallback

float32[6] y
float32[96] b
float32[6] control_allocation_scale
float32[16] actuator_scale
float32[3] error
float32[16] u
float32[16] u_ultimate
float32[16] u_phys
float32[16] u_ultimate_phys
float32[16] umin
float32[16] umax
