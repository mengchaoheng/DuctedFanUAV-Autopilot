# Disturbance estimate from the enabled translational INDI feedback path.
# The MPC consumer also checks both timestamps; a stopped publisher expires.
uint64 timestamp                     # publication time since system start [us]
uint64 timestamp_sample              # acceleration sample underlying this estimate [us]
float32[3] acceleration_disturbance   # a_f - g*e3 - beta_f, inertial NED [m/s^2]; NaN if unavailable
bool valid                           # DFB requested, translational INDI enabled, flight/feedback valid and estimate finite
