uint32 MESSAGE_VERSION = 0

uint64 timestamp                         # publication time since system start [us]
float32 thrust_acceleration              # positive collective thrust acceleration [m/s^2]
float32[3] rates                         # nominal body-rate setpoint in FRD [rad/s]
float32[3] mpc_disturbance               # acceleration disturbance already predicted by MPC in NED [m/s^2]
