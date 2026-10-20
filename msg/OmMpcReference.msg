# Latched harmonic geometry from a flight task, in local NED coordinates.
uint64 timestamp # Phase origin (publication time), not a required MPC release.
float32[3] center
float32[3] amplitude
float32[3] harmonic
float32[3] phase_offset
float32 phase
float32 phase_rate
float32 heading_rotation
float32 yaw
bool valid # Constant phase-rate tracking with fixed yaw; approach stays on PID.
