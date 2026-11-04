# Translational INDI command and the matched feedback used in this update.
# mc_att_control checks validity/freshness before selecting this command.
# Actual application is reported only by om_mpc_control_status.translational_indi_active.
uint64 timestamp          # command publication time [us]
uint64 timestamp_nominal  # timestamp_valid of the OmMpcCommand being corrected [us]
uint64 timestamp_sample   # a_f sample time [us]; 0 if unavailable
uint64 timestamp_force    # beta_f matched force sample/interpolation time [us]; 0 if unavailable

float32[3] omega_cmd      # body-FRD angular-rate command after entry transition [rad/s]
float32[3] thrust_body    # normalized body-FRD thrust command
bool valid               # correction computed successfully under the enable/flight gates; not final application

# Paper notation: beta_c = beta_f + g*e3 + beta_M + d_bar - a_f;
# omega^c = omega^M + K_phi*e_phi; omega_cmd = omega^M + eta*(omega^c - omega^M).
# OmMpcCommand.rates supplies omega^M. Unavailable feedback/error is NaN.
float32[3] a_f            # selected filtered acceleration, NED [m/s^2]
float32[3] beta_f         # filtered and time-aligned achieved thrust acceleration, NED [m/s^2]
float32[3] e_phi          # Log(R' R_c), body-FRD minimum thrust-axis rotation [rad]
