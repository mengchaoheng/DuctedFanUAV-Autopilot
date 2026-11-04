# Private MPC command. mc_att_control arbitrates and forwards to the rate loop.
uint64 timestamp
# Publication/diagnostics describe this attempt; control freshness uses the
# original successful output time below, which never advances while held.
uint64 timestamp_valid
uint64 timestamp_sample
uint64 reference_timestamp
float32[3] rates
float32 hover_thrust          # thrust scale used by this QP and its downstream INDI correction
float32[3] thrust_body
float32[3] mpc_disturbance      # d_bar used by this QP, NED [m/s^2]; downstream INDI compensates only the residual
float32[3] position_error
float32 primal_residual
int32 solver_status
uint32 iterations
# Complete MPC computation: frontend + QP update + solve/check wall time [us].
uint32 mpc_compute_us
uint32 frontend_us
uint32 update_us
# QP solve/check section only; excludes frontend and QP update [us].
uint32 qp_solve_us
# Thread CPU for the same computation; UINT32_MAX if unavailable.
uint32 cpu_us
# Cold setup cannot authorize new control values; an older valid command may
# remain held during recovery. Ineligible cycles have solver_status=0.
bool initializing
bool deadline_missed
bool active
bool held                    # control values retained after a failed/late attempt
