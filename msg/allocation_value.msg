uint64 timestamp	# time since system start(microseconds)
uint64 timestamp_sample
int8 flag #-2 mix file owns PID allocation, -1 mix file owns INDI allocation, 0-inv, 1-wls, 2-dir, 3-pca, 4-for pca when err_flag_1<0
uint8 y_dim
uint8 u_dim
uint8 MAX_Y = 6
uint8 MAX_U = 16
int8 requested_method
uint8 fallback
float32[6] y
float32[96] b		# row-major control effectiveness matrix B: b[row * MAX_U + col], same units as y and u
float32[3] error
float32[3] ud_error
float32[3] ue_error
float32[16] u		# allocated actuator command in the same units as B and U
float32[16] u_ultimate	# virtual-actuator estimate/command in the same units as B and U
float32[16] umin	# active allocation lower limits in the same units as B and U
float32[16] umax	# active allocation upper limits in the same units as B and U
