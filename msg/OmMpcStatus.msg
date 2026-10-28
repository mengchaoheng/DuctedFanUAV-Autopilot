# Final rate-setpoint arbitration status. True during OM-MPC handover/tracking.
# Published by mc_att_control after publishing a rate/thrust setpoint using MPC.
uint64 timestamp
float32 takeover_progress # 0 at PID-to-MPC entry, 1 when MPC fully replaces PID; 0 while inactive
bool active
bool acceleration_indi_active # corrected command adopted at final arbitration
