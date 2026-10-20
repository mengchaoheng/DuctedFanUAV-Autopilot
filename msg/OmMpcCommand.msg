# Private MPC command. mc_att_control arbitrates and forwards to the rate loop.
uint64 timestamp
uint64 timestamp_sample
uint64 reference_timestamp
float32[3] rates
float32[3] thrust_body
float32[3] position_error
float32 primal_residual
int32 solver_status
uint32 iterations
# All-attempt frontend + QP update + solve/check elapsed wall time.
uint32 solve_us
uint32 frontend_us
uint32 update_us
uint32 solver_us
# Thread CPU for the same computation; UINT32_MAX if unavailable.
uint32 cpu_us
# Cold setup belongs to standby PID. Ineligible cycles have solver_status=0.
bool initializing
bool deadline_missed
bool active
