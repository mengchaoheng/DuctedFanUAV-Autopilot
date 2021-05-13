uint64 timestamp				# time since system start (microseconds)
float32 propeller_omega_d
float32 propeller_omega_0
float32 dpropeller_omega_d
float32 dpropeller_omega_0
float32[4] delta				# Control deflection angle. 0 for thrust, and 1-4 for control surfaces (rad)
float32[4] last_deltacmd
