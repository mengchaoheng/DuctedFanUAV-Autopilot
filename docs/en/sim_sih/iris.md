# Project Iris SIH model

This repository provides `sihsim_iris`, airframe **22017**, in the project's
22000–22999 custom model range. It reproduces the rigid-body geometry and maximum thrust
properties of this repository's modified Gazebo Classic Iris, not the upstream
stock Iris.

```sh
make px4_sitl_sih sihsim_iris
```

On this Mac, follow `venv_readme.md` in an interactive terminal:

```sh
source ~/px4_build_env.sh
```

After the script enters its x86 shell, verify `uname -m` is `x86_64` and
`which python` is `/Users/mch/arm_env/bin/python`. In that same terminal run
`make px4_sitl_sih sihsim_iris`.

Wait for `Ready for takeoff!`, then use QGroundControl or enter these commands
at the `pxh>` prompt:

```sh
commander takeoff
```

To land:

```sh
commander land
```

SIH has no Gazebo window. QGroundControl connects on UDP 14550 and displays
the vehicle and its track. See [SIH visualization](index.md#sitl-visualization)
for optional viewers.

## Parameter mapping

The airframe sources `10015_gazebo-classic_iris` so that the existing INDI,
position control, allocation, and rotor geometry defaults remain shared.

| Quantity | SIH value / source |
| --- | --- |
| Mass | `SIH_MASS=0.75` kg |
| Inertia diagonal | `SIH_IXX/IYY/IZZ=0.0025/0.0021/0.0043` kg m² |
| Cross inertia | Zero |
| Maximum thrust per rotor | `SIH_T_MAX=8.50013845` N = `1.51e-6 × 2372.6²` |
| Motor speed time constant | `SIH_T_TAU=0.004` s, one-step response at 250 Hz |
| Motor position and reaction torque | `CA_ROTOR0..3_PX/PY/KM`, FRD coordinates |
| Linear drag / angular damping | Built-in `SIH_KDV=1`, `SIH_KDW=0.025` |
| Thrust curve | `SIH_QUAD_MODE=1`, thrust proportional to squared speed |
| Output thrust inversion | Shared `THR_MDL_FAC=1` |
| IMU and simulation rate | Default SITL SIH configuration, normally 250 Hz |

Mode 1 supports four fixed upward thrust axes and outputs ordered as motors
1–4. It uses `SIH_T_MAX` for every rotor; `CA_ROTORx_CT` remains a controller
allocation parameter. It ignores `SIH_L_ROLL`, `SIH_L_PITCH`, and `SIH_Q_MAX`.
The default `SIH_QUAD_MODE=0` retains the existing symmetric, linear-thrust
SIH quad model.

The original `read_motors()` is unchanged: the built-in first-order actuator
lag is applied before squaring the motor signal. `SIH_T_TAU=0.004` makes
`dt/tau=1` at the default 250 Hz rate, approximating Iris's fast motor response
with one-step tracking. Gazebo's 0.5 ms time constant cannot be copied into
this Euler update at a 4 ms step: `dt/tau=8` would be unstable. If changing
the simulation rate, reconsider this time constant as well.
Drag, angular damping and IMU rates retain SIH defaults. Shared Iris INDI
controller parameters are left unchanged for user tuning.

This is a single rigid-body approximation: negligible GPS/rotor-link masses
and inertias, rotor joint physics, Gazebo ground contact and sensor models
are not reproduced. The SIH ESC RPM report is illustrative, not a calibrated
Iris tachometer. SIH and Gazebo trajectories need not match sample for sample.
The shared controller's delay tuning may need adjustment for aggressive SIH
flight because the sensor pipelines differ.

Changing to this airframe triggers PX4's normal airframe parameter reset.
Later launches of the same airframe retain saved user overrides.

## Updating from the initial 1 kHz version

The initial version explicitly saved `IMU_INTEG_RATE=1000`. Removing that
line from the airframe does not remove an already saved value. To adopt the
revised airframe defaults, run the following once in the `pxh>` console while disarmed,
then shut down and relaunch:

```sh
param reset IMU_INTEG_RATE IMU_GYRO_RATEMAX SIH_T_TAU SIH_KDV SIH_KDW
param save
shutdown
```

This resets only the listed simulation settings, not controller tuning.
The earlier 1 kHz / 0.5 ms flight validation does not apply to this
250 Hz / 4 ms actuator configuration; flight tuning is left to the user.
