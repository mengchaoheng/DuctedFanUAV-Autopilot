# Final onboard control-path selection, published by mc_att_control after
# vehicle_rates_setpoint. This reports application, not command availability.
uint64 timestamp                 # status publication time since system start [us]
bool mpc_active                  # MPC path selected, including entry transition
bool translational_indi_active   # middle-layer command selected with nonzero MPC takeover progress
float32 takeover_progress        # PID-to-MPC entry progress: 0..1; 0 when inactive
