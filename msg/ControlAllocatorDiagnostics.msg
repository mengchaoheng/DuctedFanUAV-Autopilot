# Diagnostic snapshot for one completed allocate() call; no control consumers.
# Published once per call and matrix, outside the measured allocate() interval.
# uORB instance 0/1 matches allocation matrix 0/1. No publication rate limiter.
uint64 timestamp                        # publication time since system start [us]
uint64 timestamp_sample                 # sample time of the input triggering this allocation cycle [us]
# Effective configured method is in the matching control_allocator_status instance.

bool fallback                          # True when the requested allocator failed/unavailable and an inverse fallback was attempted

int8 SOLVER_STATUS_NOT_RUN = 0          # Solver was not run: INV/legacy method selected, zero command, or no solver diagnostics
int8 SOLVER_STATUS_OK = 1               # Solver ran and returned solver_err=0
int8 SOLVER_STATUS_UNAVAILABLE = -1     # Solver was rejected before execution; interpret solver_err as an adapter precondition code
int8 SOLVER_STATUS_FAILED = -2          # Solver ran but returned a nonzero error; interpret solver_err as a solver execution code

int8 solver_status                      # One of SOLVER_STATUS_* above
# solver_err depends on solver_status:
#   SOLVER_STATUS_NOT_RUN or SOLVER_STATUS_OK: 0
#   SOLVER_STATUS_UNAVAILABLE: 1=row-rank deficient, 2=active-row count unsupported,
#       3=actuator count unsupported, 4=PCA active axes are not exactly roll/pitch/yaw,
#       5=PCA higher-priority split is missing
#   SOLVER_STATUS_FAILED: -1=initial simplex unbounded/error, -2=initial feasible solution not found,
#       -3=initial simplex iteration limit, 1=final simplex unbounded/error,
#       3=final simplex iteration limit. WLS uses 1=invalid input, 2=QR failure,
#       3=active-set iteration limit.
int8 solver_err
float32 solver_rho                     # Dimensionless method-dependent solver value; LPCA/PCA direction/scale factor, 0 when unused
float32 allocation_running_time         # [us] Latest complete allocate() call time for this matrix
