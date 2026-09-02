uint64 timestamp                 # publication time since system start [us]
uint64 timestamp_sample          # angular-velocity sample used by the bridge [us]

bool acceleration_active         # acceleration INDI + lift produced this sample
bool rate_active                 # rate INDI produced this sample
bool disturbance_valid           # matched acceleration disturbance below is valid
float32[3] acceleration_disturbance # a_F - g*e3 - fT_F in inertial NED [m/s^2]
float32[3] acceleration_ned         # filtered acceleration feedback a_F [m/s^2]
float32[3] allocated_thrust_ned     # aligned normalized allocated force F_0 [norm]
float32[3] corrected_thrust_ned     # acceleration-INDI corrected force F_c [norm]
float32[3] attitude_error           # minimum-rotation lift Log(R' R_c) [rad]
float32[3] nominal_rates            # Lu MPC body-rate command [rad/s]
float32[3] corrected_rates          # command after minimum-rotation lift [rad/s]
