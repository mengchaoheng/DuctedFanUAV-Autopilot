# DuctedFanUAV Autopilot

This repository is forked from [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) and holds the PX4 flight control solution for DuctedFanUAV.

<img src="DFUAV.jpg" width="90%" />

![image](https://github.com/user-attachments/assets/3823e609-8981-4734-9921-8ac6dc98e9be)

## Feature
Development is now centered on the `df-main` branch, which tracks PX4 `main` after the `df-1.15.4` migration. The older `df-1.x.x` branches keep the PX4-versioned history, including the early `df-1.12.3` INDI and LPCA work documented in [PINDI](https://github.com/mengchaoheng/PINDI).

Compared with the upstream PX4 baseline around commit `82e3322e0cf0afc9ad640f37a0a8b639077b3fa4`, this workspace adds three connected pieces:

* INDI control: the generic angular-rate law is integrated directly into PX4 [mc_rate_control](src/modules/mc_rate_control), which keeps the normal angular-rate PID fallback and supports an explicit MC torque-allocation matrix route. The acceleration-to-thrust correction is integrated in [mc_pos_control](src/modules/mc_pos_control). The controller design follows **Full-Mode Flight Control Framework for a Ducted-Fan Tail-Sitter UAV**.
* LPCA/PCA control allocation: [ControlAllocationLPCA.cpp](src/lib/control_allocation/control_allocation/ControlAllocationLPCA.cpp) adapts INV/DP_LPCA/DPscaled_LPCA/PCA to PX4, and [pca/ControlAllocation.h](src/lib/control_allocation/control_allocation/pca/ControlAllocation.h) contains the bounded LP implementation. The allocation algorithms follow **Aircraft control allocation** and the reference implementation in [control_allocation](https://github.com/mengchaoheng/control_allocation).
* Ducted-fan effectiveness backends: [ActuatorEffectivenessDuctedFan.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFan.cpp) supports non-VTOL ducted-fan airframes, and [ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp) supports ducted-fan tailsitter VTOL. These backends provide the physical force/torque effectiveness matrices used by allocation feedback and INDI.

Supported Gazebo Classic airframes include `ductedfan2`, `ductedfan4`, `ductedfan6`, `ductedfan_mini`, `SHC09`, `SHW09_vtol`, and `tilt_multirotor`. See [airframe startup scripts](ROMFS/px4fmu_common/init.d-posix/airframes) and [SITL targets](src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake).

The simulator includes Gazebo Classic ducted-fan dynamics in [ductedfan_plugin.cpp](Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/ductedfan_plugin/ductedfan_plugin.cpp), with spline-based duct/wing aerodynamics and a control-surface moment model.

<img src="sitl_gazebo_df4.png" width="60%" />
<img src="flight_test.png" width="30%" />

## Control Allocation and INDI

The physical-unit conventions, normalized control allocation, feedback semantics, INDI laws, PCA conditions, MC/VTOL instance routing, and current applicability limits are documented in [Control Allocation and INDI Integration](CONTROL_ALLOCATION_AND_INDI.md).

## Installation
Before running this project, you need to deploy the development environment. Please refer to the [PX4 official website](https://docs.px4.io/main/en/) (`main`) to ensure that your computer (macOS/Linux) can open the default model simulation by executing the `make px4_sitl gazebo-classic` command and take off through QGC or terminal commands. It's recommended to use Ubuntu 20.04 and QGC 5.x.

> It's easy to upgrade this project to the latest version of px4, just make sure that the [Gazebo Classic environment](https://docs.px4.io/main/en/sim_gazebo_classic/#installation) is deployed in a supported ubuntu version, but we need a lot of testing before doing so.

> Due to [Ubuntu 22.04 or later with Arm64 architecture cannot install gazebo](https://github.com/osrf/gazebo_tutorials/pull/169), Gazebo Classic may not run on arm64-based Ubuntu 22 and later versions. However, AMD64 (x86-64) should still support it. As long as the PX4 official Gazebo Classic simulation can run, the code in this repository can be executed. Most users do not need to pay attention to this. We will soon migrate to the latest gz simulation.

> This repository supports both Gazebo Classic and Gazebo (gz) simulation. At present, development and testing are still mainly focused on Gazebo Classic, and the GZ models are not yet fully polished. Nevertheless, all models support gz.

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
