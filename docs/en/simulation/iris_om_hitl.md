# Iris USB HITL: Gazebo Sim / Gazebo Classic + Raspberry Pi

Both simulators use the project's CUAV V5 Nano firmware (`px4_fmu-v5_default`).
Gazebo Sim uses the existing `iris` model with the new host MAVLink bridge;
Classic uses its existing `iris_hitl` model and MAVLink plugin.
Run only one simulator/USB bridge at a time.

```text
Simulation computer: Gazebo + QGC -- USB MAVLink -- V5 Nano
                                                     |
                                               TELEM2 UART
                                                     |
                                     Raspberry Pi: DDS Agent + ROS
```

## 1. Firmware and airframe (once)

From the PX4 repository:

```bash
make px4_fmu-v5_default
make px4_fmu-v5_default upload
```

The board configuration includes `pwm_out_sim`, `uxrce_dds_client` and
`mc_om_mpc_indi`. Alternatively flash
`build/px4_fmu-v5_default/px4_fmu-v5_default.px4` with QGC custom firmware.
For project Iris in **either simulator**, set these in the PX4 console:

```sh
param set SYS_AUTOSTART 1004
param set SYS_AUTOCONFIG 1
param save
reboot
```

1004 is **Project Iris OM-MPC HITL**, with project mass/inertia, allocation and
OM defaults. The original generic Iris HITL 1001 still exists; it does not carry
these project settings. Classic ductedfan4 continues to use **1003**.
`SYS_AUTOCONFIG=1` resets most saved parameters: export settings first and
configure the companion connection after this reboot. Confirm `SYS_HITL=1`.
Use HITL with physical motors disconnected; this airframe is not for real flight.

For TELEM2 DDS, after airframe initialization:

```sh
param show MAV_*_CONFIG
param set UXRCE_DDS_CFG 102
param set SER_TEL2_BAUD 921600
param save
reboot
```

Disable only a MAVLink instance occupying TELEM2 (102), e.g.
`param set MAV_1_CONFIG 0` if that instance uses 102. Keep USB MAVLink enabled.
Wire TELEM2 TX/RX crossed to the Pi's 3.3 V UART RX/TX and connect ground; power
both devices appropriately. Enable the Pi UART and disable its serial login
console. `/dev/ttyAMA0` below is an example: use the actual UART device.
ROS `px4_msgs` must match the project's patched firmware DDS messages.

## 2A. Gazebo Sim Harmonic: setup once

On the simulation computer, with Gazebo Harmonic and its development dependencies
installed, install its Python bindings (OSRF package repository required):

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-venv
python3 -m venv --system-site-packages ~/.venvs/px4-hitl
~/.venvs/px4-hitl/bin/pip install pymavlink pyserial
cd ~/PX4-Autopilot
make px4_sitl_default
```

This build supplies the model's Gazebo plugins; it does not start SITL.
The bridge currently targets Harmonic (`gz.transport13`, `gz.msgs10`).
Retain changes inside the `Tools/simulation/gz` Git submodule when copying or
checking out the project on another computer.

### Every start

Power the flight controller and RC transmitter, close QGC, and identify USB:

```bash
ls -l /dev/serial/by-id/
```

Use the matching stable path or `/dev/ttyACM0` below. Your user needs serial-port
access (normally membership in `dialout`, followed by a new login).
On the simulation computer, terminal 1:

```bash
cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/px4_sitl_default/src/modules/simulation/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
gz sim -r Tools/simulation/gz/worlds/hitl_iris.sdf
```

For no Gazebo GUI, add `-s`. Terminal 2:

```bash
cd ~/PX4-Autopilot
~/.venvs/px4-hitl/bin/python Tools/simulation/gz/hitl_bridge.py \
  --device /dev/ttyACM0 --baudrate 921600
```

The world reuses the **same Iris model as `gz_iris` SITL**. Do not run
`make px4_sitl gz_iris` alongside it: that starts a separate software autopilot.
The bridge sends IMU/magnetometer/barometer/GPS and receives four motor commands.
It supplies the heartbeat needed to start PX4 USB MAVLink automatically.
Airframe 1004's `THR_MDL_FAC=1` and motor mapping match the bridge's default
2372.6 rad/s scaling. No additional square root is applied.

Now open QGC on the same computer, using UDP 14550 and disabling its direct USB
autoconnection. The bridge forwards MAVLink in both directions (local port 14560).
With physical RC, disable QGC's virtual joystick. Start the Pi commands below.

The bridge prints sensor and actuator rates every two seconds. With simulation
running normally, expect roughly 250 Hz IMU, 100 Hz magnetometer, 50 Hz barometer
and 5 Hz GPS. Actuator reception must be nonzero once PX4 sends HIL outputs.
A stale IMU or actuator stream for 0.2 s zeros simulated motor commands; stopping
the bridge also sends zero. Hardware simulation is real-time, not lockstep.

## 2B. Gazebo Classic: setup once

Use a computer with **Gazebo Classic 11 and its development/build dependencies**.
The SITL board configuration enables `SIMULATOR_MAVLINK` so Classic targets are
available when CMake finds Gazebo Classic.

```bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

Set `serialDevice` and `baudRate` in the relevant existing model:

| Vehicle | PX4 `SYS_AUTOSTART` | Model under `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` |
| --- | --- | --- |
| Project Iris | 1004 | `iris_hitl/iris_hitl.sdf` |
| ductedfan4 | 1003 | `ductedfan4_hitl/ductedfan4_hitl.sdf` |

For example `<serialDevice>/dev/ttyACM0</serialDevice>` and
`<baudRate>921600</baudRate>`. Both use serial HIL and disable lockstep.
Iris was updated in place to match project physical/motor parameters; retain
these changes inside the Classic Git submodule. The df4 model/airframe are retained.

### Every start

Power the flight controller and RC transmitter, close QGC, then:

```bash
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  "$PWD" "$PWD/build/px4_sitl_default"
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
```

For df4, replace the last command with:

```bash
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_ductedfan4.world
```

Open QGC after the simulator owns USB; use its UDP 14550 forwarding and disable
QGC USB autoconnection/virtual joystick as above. Classic includes its own serial
bridge: do not start `hitl_bridge.py`. No PX4 SITL process is needed for HITL.
The upstream [PX4 HITL instructions](https://docs.px4.io/main/en/simulation/hitl)
describe this Classic workflow; the Gazebo Sim bridge above is project-specific.

## 3. Raspberry Pi: every start, for either simulator

Over SSH, terminal 1:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Terminal 2, with this ROS workspace already built:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_sensor_combined/install/setup.bash
ros2 launch geometric_controller geometric_controller.launch.py \
  launch_rviz:=false launch_tuning_panel:=false
```

The controller node generates the reference trajectory internally. No separate
trajectory process or graphical desktop is needed. Defaults select Iris, OM
(mode 8), and automatic Offboard/arm/takeoff when ready. To choose trajectory,
controller and tuning before startup, copy `config/controller.yaml`, edit it, and
add `param_file:=$HOME/controller_hardware.yaml`. For df4 or real flight also
supply a matching `vehicle_param_file`; do not use Iris vehicle constants.
Set `auto_start: false` in the copied YAML if you want to stage startup manually.
The ROS README documents runtime parameter changes and MPC tuning.

ROS supports all controller selections through this entry point. Switching ROS
modes does **not** change these required PX4 settings:

| ROS mode | PX4 settings |
| --- | --- |
| 6, Lu + PID | `MC_INDI_RATE_EN=0`; RC12 must not request native rate INDI |
| 8, OM | `MC_OM_INDI_EN=1`, `MC_OM_ACC_EN=1`, `MC_OM_DIST_EN=1`, `MC_INDI_RATE_EN=1` |

Enable OM before reboot, or start `mc_om_mpc_indi` manually if it was disabled
at boot. Verify PX4 `uxrce_dds_client status`, `pwm_out_sim status`,
`listener vehicle_local_position` and, for OM, `mc_om_mpc_indi status`.
DDS must connect and synchronize; check received feedback rates over UART.

For real flight the Pi entry point is the same. The flight controller uses the
actual vehicle's non-HITL airframe (`SYS_HITL=0`), vehicle parameters and real
sensors; the simulation computer and USB simulation bridge are absent.

## Validation scope

Firmware build and automated bridge checks pass. The new bridge has also driven
the real Gazebo Harmonic Iris model using MAVLink over a pseudo-terminal,
including sensor conversion, motor scaling, disarming, watchdog and QGC forwarding.
Run host checks with `python Tools/simulation/gz/test_hitl_bridge.py` in the venv.
Actual V5 Nano USB timing and closed-loop ROS flight remain to be tested on hardware.
Classic is configured but was not run on this development host, which lacks Classic.
