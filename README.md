# DuctedFanUAV Autopilot

This repository is forked from [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) and holds the PX4 flight control solution for DuctedFanUAV.

<img src="DFUAV.jpg" width="90%" />

![image](https://github.com/user-attachments/assets/3823e609-8981-4734-9921-8ac6dc98e9be)

## Feature
Development is now centered on the `df-main` branch, which tracks PX4 `main` after the `df-1.15.4` migration. The older `df-1.x.x` branches keep the PX4-versioned history, including the early `df-1.12.3` INDI and LPCA work documented in [PINDI](https://github.com/mengchaoheng/PINDI).

* Ducted-fan hover-rate control with a standard PID fallback and an optional INDI angular-rate loop in [src/modules/df_hover_rate_control](src/modules/df_hover_rate_control).
* Unitized control allocation, with theory and reference implementation in [control_allocation](https://github.com/mengchaoheng/control_allocation), integrated into [src/modules/control_allocator](src/modules/control_allocator) and [src/lib/control_allocation](src/lib/control_allocation). The PX4 adapter is in [ControlAllocationLPCA.cpp](src/lib/control_allocation/control_allocation/ControlAllocationLPCA.cpp), while the core INV/DP_LPCA/DPscaled_LPCA/PCA solver is in [pca/ControlAllocation.h](src/lib/control_allocation/control_allocation/pca/ControlAllocation.h). It also includes priority-split torque allocation, allocator diagnostics, and INDI feedback through [AllocationValue](msg/AllocationValue.msg) and [VehicleTorqueSetpoint](msg/VehicleTorqueSetpoint.msg).
* Ducted-fan actuator-effectiveness backends for hover and tailsitter VTOL in [ActuatorEffectivenessDuctedFan.hpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFan.hpp) and [ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp).
* Supported Gazebo Classic airframes: `ductedfan2`, `ductedfan4`, `ductedfan6`, `ductedfan_mini`, `SHC09`, `SHW09_vtol`, and `tilt_multirotor`. See [airframe startup scripts](ROMFS/px4fmu_common/init.d-posix/airframes) and [SITL targets](src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake).
* Integrated Gazebo Classic ducted-fan dynamics in [ductedfan_plugin.cpp](Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/ductedfan_plugin/ductedfan_plugin.cpp), with spline-based duct/wing aerodynamics and a control-surface moment model.
* Extensive Pixhawk-based flight testing on DuctedFanUAV prototypes.

<img src="sitl_gazebo_df4.png" width="60%" />
<img src="flight_test.png" width="30%" />

## Methodology
Start from the physical allocation model:

```text
v = B_phys u_phys
u_phys = U u_norm
v_norm = D v = D B_phys U u_norm = B_norm u_norm
```

Here `D = diag(_control_allocation_scale)` unitizes the control axes, and `U` converts normalized actuator commands to physical actuator units. PX4's built-in PID path already outputs `v_norm`, so its output is sent directly to allocation. INDI instead computes a physical angular-acceleration command `v`, so it must publish `v_norm = D v`.

PX4's built-in pseudo-inverse/sequential-desaturation allocation receives `B` in `_effectiveness`, computes the mixer `M = B^+`, and then, when `_metric_allocation=false`, unitizes the model. Because the implementation stores the inverse mixer, this appears as column scaling by `D^{-1}`:

```text
B_norm = D B
M_norm = B^+ D^{-1}
u_norm = M_norm y
y_hat = D B u_norm
```

The key switches are `_metric_allocation` and `_normalize_rpy`: `_normalize_rpy` decides whether roll/pitch/yaw are unitized, thrust axes are unitized by PX4's thrust logic, and `_normalization_needs_update` only controls when `D` is recomputed. `CA_AIRFRAME` selects the effectiveness backend that supplies `_normalize_rpy` through `getNormalizeRPY()`. Our ducted-fan airframes enable RPY unitization.

Our LPCA/PCA allocation keeps the same normalized PX4 contract, but applies the scale to the forward model because the solver works on `B`, not on `B^+`. [ControlAllocator.cpp](src/modules/control_allocator/ControlAllocator.cpp) first builds `allocation_matrix = B_phys U`, with `U` from `_allocation_actuator_scale` (`DF_CS_MAX` for servos, `DF_MOT_MAX` for motors). [ControlAllocationLPCA.cpp](src/lib/control_allocation/control_allocation/ControlAllocationLPCA.cpp) has its own `_metric_allocation` and `_normalization_needs_update`, computes the same `D`, builds `_effectiveness_unit`, and sends the bounded problem to INV/DP_LPCA/DPscaled_LPCA/PCA in [pca/ControlAllocation.h](src/lib/control_allocation/control_allocation/pca/ControlAllocation.h):

```text
B_norm = _effectiveness_unit = D B_phys U
find u_norm within actuator bounds so B_norm u_norm tracks y
```

INDI closes the loop in physical angular-acceleration coordinates before this normalization. [DfHoverRateControl.cpp](src/modules/df_hover_rate_control/DfHoverRateControl.cpp) computes `v` from the equilibrium `B_phys`, publishes `v_norm = D v`, and splits it into lower-priority rate-error feedback plus higher-priority INDI feedback in [VehicleTorqueSetpoint](msg/VehicleTorqueSetpoint.msg). PID can survive arbitrary effectiveness magnitudes after PX4 unitization; INDI is only correct when `B_phys`, `DF_CS_MAX`, `DF_MOT_MAX`, and `DF_ACC_MASS` have the right physical units.

The simulator follows the same model. [ductedfan_plugin.cpp](Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/ductedfan_plugin/ductedfan_plugin.cpp) computes FRD forces and moments from rotor speed, wind-relative airspeed, angle of attack, duct/wing splines, rotor torque, gyroscopic terms, and `B_cs`, then converts frames only at the Gazebo boundary.

## Installation
Before running this project, you need to deploy the development environment. Please refer to the [PX4 official website](https://docs.px4.io/main/en/) (`main`) to ensure that your computer (macOS/Linux) can open the default model simulation by executing the `make px4_sitl gazebo-classic` command and take off through QGC or terminal commands. It's recommended to use Ubuntu 20.04 and QGC 5.x.

> It's easy to upgrade this project to the latest version of px4, just make sure that the [Gazebo Classic environment](https://docs.px4.io/main/en/sim_gazebo_classic/#installation) is deployed in a supported ubuntu version, but we need a lot of testing before doing so.

> Due to [Ubuntu 22.04 or later with Arm64 architecture cannot install gazebo](https://github.com/osrf/gazebo_tutorials/pull/169), Gazebo Classic may not run on arm64-based Ubuntu 22 and later versions. However, AMD64 (x86-64) should still support it. As long as the PX4 official Gazebo Classic simulation can run, the code in this repository can be executed. Most users do not need to pay attention to this. We will soon migrate to the latest gz simulation.

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/main/en/#support) if you need help!

For Ubuntu 20.04, installing the simulation environment is quite straightforward:
1. Install git:

```bash
sudo apt install git
```

2. Clone code:

```bash
git clone https://github.com/mengchaoheng/DuctedFanUAV-Autopilot --recursive
```

3. Go to the path of the code:

```bash
cd DuctedFanUAV-Autopilot
```

4. Run the ubuntu.sh with no arguments (in a bash shell) to install everything:

```bash
# For arm64-based ubuntu. See https://github.com/PX4/PX4-Autopilot/issues/21117
bash ./Tools/setup/ubuntu.sh
```

Or download the development environment deployment script from the official website.

```bash
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
bash ubuntu.sh
```

5. Start Gazebo SITL using the following command:


5.1 Test the built-in quadcopter simulation:

```bash
make px4_sitl gazebo-classic
```
5.2 Test the simulation of this project:
```bash
make px4_sitl gazebo-classic_ductedfan4
```

> **Note:**  In Ubuntu 22.04 and higher versions, Gazebo Classic is no longer supported on arm64 Ubuntu. If Gazebo was installed using a script on amd64 Ubuntu, it needs to be uninstalled and reinstalled:
```bash
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```

## Usage
Clone this repository:
```
git clone https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git

cd DuctedFanUAV-Autopilot
```

Make sure you're on the `df-main` branch. You can use `git status` to check it.
```
git checkout df-main
```

Ensure that the required submodules for loading the `df-main` branch are loaded.
```
git submodule update --init --recursive
```

> Note: If submodule update error, first switch to `main`, then run the above command, and then switch back to `df-main`, and then run the aforementioned command to update submodules.

When switching branches or wishing to recompile, you can use
```
make distclean
```
to keep a clean compilation, and then run
```
git submodule update --init --recursive

```
again to rebuild, The compilation command is as follows.

> **Note:** px4 is not sensitive to the Python environment, but you need to ensure that you have installed the required Python packages. Refer to [Development Environment Deployment](https://docs.px4.io/main/en/dev_setup/dev_env)
### Simulation
1. ductedfan2
```
make px4_sitl gazebo-classic_ductedfan2
```
2. ductedfan4
```
make px4_sitl gazebo-classic_ductedfan4
```
3. ductedfan6
```
make px4_sitl gazebo-classic_ductedfan6
```
4. ductedfan_mini
```
make px4_sitl gazebo-classic_ductedfan_mini
```
5. SHC09
```
make px4_sitl gazebo-classic_SHC09
```
6. SHW09_vtol
```
make px4_sitl gazebo-classic_SHW09_vtol
```
7. Multirotor with tilt
```
make px4_sitl gazebo-classic_tilt_multirotor
```
### Flight with pixhawk

Taking pixhawk 4 as an example, the upload command is:

```
make px4_fmu-v5 upload
```
Other versions are similar, please refer to the official website for more details.
