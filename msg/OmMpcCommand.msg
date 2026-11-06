# Private MPC command. mc_att_control arbitrates and forwards to the rate loop.
uint64 timestamp            # publication time [us]
uint64 timestamp_valid      # time the carried command was first accepted [us]; 0 when valid is false
uint64 timestamp_sample     # local-position sample time for this attempt [us]; 0 if unavailable
uint64 reference_timestamp  # trajectory reference publication time for this attempt [us]; 0 if unavailable
float32[3] rates             # commanded angular rates in body FRD [rad/s]
float32 hover_thrust          # thrust scale used by this QP and its downstream INDI correction
float32[3] thrust_body       # normalized thrust in body FRD; upward thrust is negative Z
float32[3] mpc_disturbance      # d_bar used by this QP, NED [m/s^2]; downstream INDI compensates only the residual
float32[3] position_error    # current minus reference position in NED [m] when a new command is formed
float32 primal_residual     # largest initial-state, input-bound or dynamics violation in the QP result
int32 solver_status         # status of this attempt; left at 0 when no QP attempt is made
uint32 iterations           # solver iterations for this attempt, including recovery if run
# Elapsed time from eligibility through final command selection, before publication [us].
# Zero if no QP attempt was made.
uint32 mpc_compute_us
# This attempt used the cold-start path to prepare the solver workspace.
# A cold attempt cannot supply a new control command.
bool initializing
# The release budget was already exhausted before solving, or the QP attempt exceeded its deadline.
bool deadline_missed
# This message carries a usable MPC command, either new or held.
# Actual control-path selection is reported by om_mpc_control_status.mpc_active.
bool valid
# The command comes from a previous successful attempt and retains its original timestamp_valid.
bool held
