uint64 timestamp                 # publication time since system start [us]
uint64 timestamp_sample          # angular-velocity sample used by the bridge [us]

bool acceleration_active         # acceleration INDI + lift produced this sample
bool rate_active                 # rate INDI produced this sample
bool disturbance_valid           # matched acceleration disturbance below is valid
float32[3] acceleration_disturbance # a_F - g*e3 - fT_F in inertial NED [m/s^2]
