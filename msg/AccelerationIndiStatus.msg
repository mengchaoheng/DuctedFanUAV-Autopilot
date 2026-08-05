uint64 timestamp                         # publication time [us]
uint64 timestamp_sample                  # local-position sample driving this update [us]
uint64 force_timestamp                   # selected delayed force sample/interpolation time [us]

float32[3] acceleration_feedback         # filtered a_0 used by acceleration INDI, NED [m/s^2]
float32[3] acceleration_velocity_derivative # source 0: velocity-filter/difference/filter path, NED [m/s^2]
float32[3] acceleration_ekf              # source 1: filtered EKF local-position acceleration, NED [m/s^2]
float32[3] acceleration_imu              # source 2: filtered body IMU acceleration rotated to NED with gravity removed [m/s^2]
float32[3] allocated_thrust_acceleration # delayed and scaled F_0/m used by INDI, NED [m/s^2]
float32[3] allocated_thrust_acceleration_undelayed # newest scaled F_0/m before pure delay, NED [m/s^2]
float32[3] raw_thrust_setpoint            # selected controller output before bumpless transfer and common limits

float32 hover_thrust                     # HTE/MPC hover thrust currently applied [normalized thrust]
float32 force_feedback_scale             # runtime multiplier applied to allocation force
float32 controller_transition_progress   # 0 at PID/INDI transfer start, 1 when complete
float32 hover_thrust_transition_progress # 0 at HTE acquisition start, 1 when complete

uint8 acceleration_source                # requested MPC_INDI_A_SRC value
uint8 acceleration_source_valid          # validity bit mask: bit 0 velocity derivative, bit 1 EKF, bit 2 IMU
bool feedback_valid                      # delayed F_0/m and a_0 are finite
bool controller_transition_active        # PID/INDI bumpless transfer is in progress
bool hover_thrust_transition_active      # first-valid HTE model transfer is in progress
