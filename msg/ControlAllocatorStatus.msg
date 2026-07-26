uint64 timestamp                        # time since system start (microseconds)

bool fallback                          # True when requested DP_LPCA, DPscaled_LPCA, or PCA could not be used and INV was attempted instead

int8 SOLVER_STATUS_NOT_RUN = 0          # LP solver was not run: INV/legacy method selected, zero command, or no solver diagnostics
int8 SOLVER_STATUS_OK = 1               # LP solver ran and returned solver_err=0
int8 SOLVER_STATUS_UNAVAILABLE = -1     # LP solver was rejected before execution; interpret solver_err as an adapter precondition code
int8 SOLVER_STATUS_FAILED = -2          # LP solver ran but returned a nonzero error; interpret solver_err as a solver execution code

int8 solver_status                      # One of SOLVER_STATUS_* above
# solver_err depends on solver_status:
#   SOLVER_STATUS_NOT_RUN or SOLVER_STATUS_OK: 0
#   SOLVER_STATUS_UNAVAILABLE: 1=row-rank deficient, 2=active-row count unsupported,
#       3=actuator count unsupported, 4=PCA active axes are not exactly roll/pitch/yaw,
#       5=PCA higher-priority split is missing
#   SOLVER_STATUS_FAILED: -1=initial simplex unbounded/error, -2=initial feasible solution not found,
#       -3=initial simplex iteration limit, 1=final simplex unbounded/error,
#       3=final simplex iteration limit
int8 solver_err
bool full_row_rank                     # True if the normalized active-row effectiveness matrix has full row rank; false/default for methods without these diagnostics
float32 solver_rho                     # Dimensionless LPCA/PCA objective direction/scale factor returned by the solver; method-dependent, 0 when not run
float32 solver_prepare_time            # [us] Latest LPCA adapter work before constructing pca/ControlAllocation.h solver objects; 0 when not run
float32 solver_core_time               # [us] Latest solver-object construction, simplex solve, and restoring; 0 when not run
float32 solver_post_time               # [us] Latest adapter validation and output copy after the solver returns; 0 when not run

bool torque_setpoint_achieved           # Boolean indicating whether the 3D torque setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.
float32[3] unallocated_torque           # Unallocated torque. Equal to 0 if the setpoint was achieved.
                                        # Computed as: unallocated_torque = torque_setpoint - allocated_torque

bool thrust_setpoint_achieved           # Boolean indicating whether the 3D thrust setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.
float32[3] unallocated_thrust           # Unallocated thrust. Equal to 0 if the setpoint was achieved.
                                        # Computed as: unallocated_thrust = thrust_setpoint - allocated_thrust

int8 ACTUATOR_SATURATION_OK        =  0 # The actuator is not saturated
int8 ACTUATOR_SATURATION_UPPER_DYN =  1 # The actuator is saturated (with a value <= the desired value) because it cannot increase its value faster
int8 ACTUATOR_SATURATION_UPPER     =  2 # The actuator is saturated (with a value <= the desired value) because it has reached its maximum value
int8 ACTUATOR_SATURATION_LOWER_DYN = -1 # The actuator is saturated (with a value >= the desired value) because it cannot decrease its value faster
int8 ACTUATOR_SATURATION_LOWER     = -2 # The actuator is saturated (with a value >= the desired value) because it has reached its minimum value

int8[16] actuator_saturation            # Indicates actuator saturation status.
                                        # Note 1: actuator saturation does not necessarily imply that the thrust setpoint or the torque setpoint were not achieved.
                                        # Note 2: an actuator with limited dynamics can be indicated as upper-saturated even if it as not reached its maximum value.

uint16 handled_motor_failure_mask        # Bitmask of failed motors that were removed from the allocation / effectiveness matrix. Not necessarily identical to the report from FailureDetector
uint16 motor_stop_mask                   # Bitmaks of motors stopped by failure injection

float32 allocation_running_time         # [us] Latest complete allocate() call time for this matrix
float32 allocation_running_time_avg     # [us] Online mean allocate() call time since the allocation method was created or reset
uint32 allocation_running_time_samples  # Number of allocate() calls accumulated in allocation_running_time_avg
