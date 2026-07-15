uint64 timestamp
uint64 timestamp_sample

uint8 MAX_U = 16

uint8 num_actuators                    # Valid columns in effectiveness_bu and valid entries in u
uint8 feedback_axes_mask              # Bit i is set when the solver matrix retains authority on body-FRD axis i
bool fallback                          # Requested LPCA/PCA method fell back to INV

int8 SOLVER_STATUS_NOT_RUN = 0
int8 SOLVER_STATUS_OK = 1
int8 SOLVER_STATUS_UNAVAILABLE = -1
int8 SOLVER_STATUS_FAILED = -2

int8 solver_status
int8 solver_err
bool full_row_rank
float32 solver_rho
float32 solver_prepare_time # [us] LPCA adapter work before entering pca/ControlAllocation.h objects
float32 solver_core_time # [us] pca/ControlAllocation.h construction, solver and restoring
float32 solver_post_time # [us] LPCA adapter work after returning from pca/ControlAllocation.h calls

float32[96] effectiveness_bu       # Body-FRD B*U, row-major; valid at row*MAX_U+actuator for actuator<num_actuators
float32[6] allocated_wrench        # Filtered per-matrix body-FRD (B*U)*(u_sp-u_trim); rotor thrust follows F=-T*b_z, [N m, N]
float32[6] control_allocation_scale # D: physical wrench to normalized control setpoint
float32[16] u                     # Normalized actuator allocation variable relative to trim
