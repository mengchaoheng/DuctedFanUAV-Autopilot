# Iris USB HITL: Gazebo Classic + Raspberry Pi

Use the project CUAV V5 Nano firmware (`px4_fmu-v5_default`), the existing
Classic `iris_hitl` model and its MAVLink plugin.

```text
Simulation computer: Classic + QGC -- USB MAVLink -- V5 Nano
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
For project Iris HITL, set these in the PX4 console:

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

## 2. Gazebo Classic: setup once

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
QGC direct USB autoconnection. With physical RC, disable QGC's virtual joystick.
Classic includes its own serial bridge. No PX4 SITL process is needed for HITL.
See the upstream [PX4 HITL instructions](https://docs.px4.io/main/en/simulation/hitl).

## 3. Raspberry Pi: every start

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
