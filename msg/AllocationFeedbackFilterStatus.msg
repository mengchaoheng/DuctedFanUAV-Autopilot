uint64 timestamp
uint64 filter_event_timestamp        # Most recent reset or coefficient reconfiguration

float32 sample_interval              # Mean ControlAllocator dt used to estimate the sample rate [s]
float32 filter_sample_rate_hz        # Sample rate used for the current coefficients [Hz]
float32 torque_filter_cutoff_hz      # Actual allocated-torque filter cutoff, 0 if bypassed [Hz]
float32 force_filter_cutoff_hz       # Actual allocated-force filter cutoff, 0 if bypassed [Hz]

uint32 filter_reset_count            # Cumulative feedback-filter state reset count
uint32 filter_reconfigure_count      # Cumulative coefficient configuration count
uint8 filter_event_reason            # Most recent reset or reconfiguration reason
bool filters_configured

uint8 FILTER_EVENT_INITIAL = 1
uint8 FILTER_EVENT_PARAMETER = 2
uint8 FILTER_EVENT_SAMPLE_RATE = 4
uint8 FILTER_EVENT_GAP = 16
uint8 FILTER_EVENT_AXIS_RECOVERY = 32
