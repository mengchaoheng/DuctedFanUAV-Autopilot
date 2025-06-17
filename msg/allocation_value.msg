uint64 timestamp	# time since system start(microseconds)
uint64 timestamp_sample
int8 flag #0-inv, 1-wls, 2-dir, 3-pca  4-for pca when err_flag_1<0
float32[3] error
float32[3] ud_error
float32[3] ue_error
float32[4] u
float32[4] u_ultimate
float32[4] umin
float32[4] umax



