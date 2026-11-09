uint64 timestamp                        # time since system start (microseconds)

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

uint16 handled_motor_failure_mask       # Bitmask of failed motors that were removed from the allocation / effectiveness matrix. Not necessarily identical to the report from FailureDetector
uint16 motor_stop_mask                  # Bitmaks of motors stopped by failure injection

bool actuator_group_preflight_check_active   # True while an actuator group preflight check (VEHICLE_CMD_ACTUATOR_GROUP_TEST) is overriding the torque/thrust setpoint or collective-tilt

# Effective configured method for this uORB allocation-matrix instance.
# AUTO and airframe-specific routing are resolved; a per-call fallback does not change this field.
# Per-call solver/fallback/timing diagnostics are in control_allocator_diagnostics.
int8 allocation_method

int8 METHOD_NONE = -1
int8 METHOD_PSEUDO_INVERSE = 0
int8 METHOD_SEQUENTIAL_DESATURATION = 1
int8 METHOD_AUTO = 2
int8 METHOD_INV = 3
int8 METHOD_DP_LPCA = 4
int8 METHOD_DPSCALED_LPCA = 5
int8 METHOD_PCA = 6
int8 METHOD_WLS = 7
