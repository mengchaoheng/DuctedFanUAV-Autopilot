# Shared allocation feedback-filter configuration and events. One instance.
# Published on changes and as a 10 Hz snapshot; no control consumers.
uint64 timestamp                    # status publication time since system start [us]
uint64 filter_event_timestamp       # latest filter reset or coefficient reconfiguration time [us]
float32 filter_sample_rate_hz       # coefficient sample rate [Hz]; NaN if unconfigured
float32 torque_filter_cutoff_hz     # actual torque cutoff [Hz]; 0 if bypassed/unconfigured
float32 force_filter_cutoff_hz      # actual force cutoff [Hz]; 0 if bypassed/unconfigured
uint32 filter_reset_count           # cumulative explicit filter resets and axis recoveries
uint32 filter_reconfigure_count     # cumulative coefficient configurations
uint8 filter_event_reason           # reason for the latest filter reset/reconfiguration

uint8 FILTER_EVENT_INITIAL = 1
uint8 FILTER_EVENT_PARAMETER = 2
uint8 FILTER_EVENT_SAMPLE_RATE = 4
uint8 FILTER_EVENT_GAP = 16
uint8 FILTER_EVENT_AXIS_RECOVERY = 32
