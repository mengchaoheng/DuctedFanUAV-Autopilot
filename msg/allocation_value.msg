uint64 timestamp	# time since system start(microseconds)
uint64 timestamp_sample
uint64 control_timestamp_sample	# timestamp_sample of actuator_controls_0 used by this mixer update
uint64 mixer_update_count	# cumulative normal mixer update count
uint64 mixer_dt_us	# raw elapsed time since the previous mixer update
uint64 mixer_long_dt_count	# cumulative mixer updates with raw dt >= 2000 us
uint64 filter_reset_count	# cumulative filter resets; timing resets occur only when chain_fix_enabled is false
uint64 sample_freq_change_count	# cumulative changes of mixer_sample_freq_hz greater than 1 percent
uint32 control_generation	# actuator_controls_0 uORB generation copied by the mixer
uint32 topic_update_interval_us	# configured actuator_controls subscription interval
float32 mixer_sample_freq_hz	# instantaneous mixer frequency used by actuator model
float32 filter_cutoff_hz	# active low-pass cutoff
float32 actuator_time_constant_s	# effective first-order actuator-estimator time constant
bool actuator_simulation_enabled	# true: estimate is sent; false: raw allocation is sent
bool chain_fix_enabled	# true: recursive estimate and variable-dt state-preserving filter
int8 flag #0-inv, 1-wls, 2-dir, 3-pca  4-for pca when err_flag_1<0
float32[3] error
float32[3] ud_error
float32[3] ue_error
float32[4] u
float32[4] u_ultimate
float32[4] umin
float32[4] umax
