uint64 timestamp

float32[6] allocated_wrench         # Filtered per-matrix body-FRD (B*U)*(u_sp-u_trim), [N m, N]
float32[6] control_allocation_scale # D: physical wrench to normalized control setpoint
