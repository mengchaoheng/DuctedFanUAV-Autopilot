uint64 timestamp
uint64 timestamp_sample

uint8 MAX_Y = 6
uint8 MAX_U = 16

uint8 y_dim
uint8 u_dim
bool feedback_valid
int8 requested_method
uint8 fallback

int8 SOLVER_STATUS_NOT_RUN = 0
int8 SOLVER_STATUS_OK = 1
int8 SOLVER_STATUS_ACCEPTED_WITH_ERROR = 2
int8 SOLVER_STATUS_UNAVAILABLE = -1
int8 SOLVER_STATUS_FAILED = -2

int8 solver_status
int8 solver_err
bool full_row_rank
bool priority_split_valid
uint8 active_rows
uint8 active_axes_mask
float32 solver_rho
float32 solver_residual
float32 solver_tolerance
float32 solver_prepare_time # [us] LPCA adapter work before entering pca/ControlAllocation.h objects
float32 solver_core_time # [us] pca/ControlAllocation.h construction, solver and restoring
float32 solver_post_time # [us] LPCA adapter work after returning from pca/ControlAllocation.h calls
float32 allocation_running_time # [us] latest allocation calculation time
float32 allocation_running_time_avg # [us] online average allocation calculation time
uint32 allocation_running_time_samples

float32[6] y
float32[96] b
float32[6] control_allocation_scale
float32[16] actuator_scale
float32[3] error
float32[16] u
float32[16] u_ultimate
float32[16] u_phys
float32[16] u_ultimate_phys
float32[16] umin
float32[16] umax
