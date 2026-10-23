# Optional acceleration-INDI correction. mc_att_control remains the only
# onboard MPC publisher of vehicle_rates_setpoint.
uint64 timestamp
uint64 timestamp_nominal   # timestamp of the OmMpcCommand being corrected
float32[3] rates
float32[3] thrust_body
bool active
