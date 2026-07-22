# DuctedFanUAV Autopilot

This repository is forked from [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) and holds the PX4 flight control solution for DuctedFanUAV.

<img src="DFUAV.jpg" width="90%" />

![image](https://github.com/user-attachments/assets/3823e609-8981-4734-9921-8ac6dc98e9be)

## Feature
Development is now centered on the `df-main` branch, which tracks PX4 `main` after the `df-1.15.4` migration. The older `df-1.x.x` branches keep the PX4-versioned history, including the early `df-1.12.3` INDI and LPCA work documented in [PINDI](https://github.com/mengchaoheng/PINDI).

Compared with the upstream PX4 baseline around commit `82e3322e0cf0afc9ad640f37a0a8b639077b3fa4`, this workspace adds four connected pieces:

* INDI control: the generic angular-rate law is integrated directly into PX4 [mc_rate_control](src/modules/mc_rate_control), which keeps the normal angular-rate PID fallback and supports an explicit MC torque-allocation matrix route. The acceleration-to-thrust correction is integrated in [mc_pos_control](src/modules/mc_pos_control). The controller design follows **Full-Mode Flight Control Framework for a Ducted-Fan Tail-Sitter UAV**.
* Selectable attitude errors: [mc_att_control](src/modules/mc_att_control) adds the `MC_ATT_ERR_MODE` parameter for selecting among eight attitude-error formulations, including the PX4 reduced-attitude method, quaternion and SO(3) logarithm methods, DCM vee error, Tal-Karaman incremental error, tilt-prioritized quaternion error, and Yu tilt-torsion error. The default value `0` preserves the original PX4 behavior.
* LPCA/PCA control allocation: [ControlAllocationLPCA.cpp](src/lib/control_allocation/control_allocation/ControlAllocationLPCA.cpp) adapts INV/DP_LPCA/DPscaled_LPCA/PCA to PX4, and [pca/ControlAllocation.h](src/lib/control_allocation/control_allocation/pca/ControlAllocation.h) contains the bounded LP implementation. The allocation algorithms follow **Aircraft control allocation** and the reference implementation in [control_allocation](https://github.com/mengchaoheng/control_allocation).
* Ducted-fan effectiveness backends: [ActuatorEffectivenessDuctedFan.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFan.cpp) supports non-VTOL ducted-fan airframes, and [ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp) supports ducted-fan tailsitter VTOL. These backends provide the physical force/torque effectiveness matrices used by allocation feedback and INDI.

Both Gazebo Classic and Gazebo (gz) support `ductedfan2`, `ductedfan4`, `ductedfan6`, `ductedfan_mini`, `SHC09`, and `SHW09_vtol`. The tilted vehicle is named `tilt_multirotor` in Gazebo Classic and `tiltrotor` in gz. See the [airframe startup scripts](ROMFS/px4fmu_common/init.d-posix/airframes), [Gazebo Classic SITL targets](src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake), and [gz models](Tools/simulation/gz/models).

The simulator includes Gazebo Classic ducted-fan dynamics in [ductedfan_plugin.cpp](Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/ductedfan_plugin/ductedfan_plugin.cpp), with spline-based duct/wing aerodynamics and a control-surface moment model.

<img src="sitl_gazebo_df4.png" width="60%" />
<img src="flight_test.png" width="30%" />

## Control Allocation and INDI

The physical-unit conventions, normalized control allocation, feedback semantics, INDI laws, PCA conditions, MC/VTOL instance routing, and current applicability limits are documented in [Control Allocation and INDI Integration](CONTROL_ALLOCATION_AND_INDI.md).

## Installation
Before running this project, deploy the development environment by following the [PX4 User Guide](https://docs.px4.io/main/en/) (`main`). Verify that your computer can launch a default PX4 vehicle and take off through QGC or terminal commands using either simulation backend:

```bash
# Gazebo Classic
make px4_sitl gazebo-classic

# Gazebo (gz)
make px4_sitl gz_x500
```

Gazebo Classic is recommended on Ubuntu 20.04, while Gazebo (gz) is recommended on Ubuntu 22.04. QGC 5.x is recommended for both backends.

> Before upgrading this project to a newer PX4 version, ensure that either the [Gazebo Classic environment](https://docs.px4.io/main/en/sim_gazebo_classic/#installation) or the [Gazebo (gz) environment](https://docs.px4.io/main/en/sim_gazebo_gz/) works with the corresponding default PX4 simulation. The project-specific models still require regression testing after an upgrade.

> Due to [Ubuntu 22.04 or later with Arm64 architecture cannot install gazebo](https://github.com/osrf/gazebo_tutorials/pull/169), Gazebo Classic may not run on Arm64-based Ubuntu 22.04 and later. Use Ubuntu 20.04 for Gazebo Classic or Ubuntu 22.04 with Gazebo (gz). AMD64 systems may still support Gazebo Classic on newer Ubuntu releases.

> This repository supports both Gazebo Classic and Gazebo (gz) simulation. At present, development and testing are still mainly focused on Gazebo Classic, and the GZ models are not yet fully polished. Nevertheless, all models support gz.

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/main/en/#support) if you need help!

The following setup applies to Ubuntu 20.04 for Gazebo Classic and Ubuntu 22.04 for Gazebo (gz):
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

5. Test the built-in quadcopter simulation:

```bash
# Ubuntu 20.04: Gazebo Classic
make px4_sitl gazebo-classic

# Ubuntu 22.04: Gazebo (gz)
make px4_sitl gz_x500
```

6. Test the DuctedFan4 simulation:

```bash
# Ubuntu 20.04: Gazebo Classic
make px4_sitl gazebo-classic_ductedfan4

# Ubuntu 22.04: Gazebo (gz)
make px4_sitl gz_ductedfan4
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

When switching branches, follow the [PX4 source-tree switching procedure](https://docs.px4.io/main/en/contribute/git_examples#changing-source-trees):

1. Clean up the current branch, de-initializing its submodules and removing all build artifacts:

   ```
   make clean
   make distclean
   ```

2. Switch to the new branch or tag. For example:

   ```
   git fetch upstream PR_test_branch
   git checkout PR_test_branch
   ```

3. Get the submodules for the new branch:

   ```
   make submodulesclean
   ```

> **Note:** px4 is not sensitive to the Python environment, but you need to ensure that you have installed the required Python packages. Refer to [Development Environment Deployment](https://docs.px4.io/main/en/dev_setup/dev_env)
### Simulation
1. ductedfan2
```
make px4_sitl gazebo-classic_ductedfan2
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_ductedfan2
```
2. ductedfan4
```
make px4_sitl gazebo-classic_ductedfan4
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_ductedfan4
```
3. ductedfan6
```
make px4_sitl gazebo-classic_ductedfan6
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_ductedfan6
```
4. ductedfan_mini
```
make px4_sitl gazebo-classic_ductedfan_mini
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_ductedfan_mini
```
5. SHC09
```
make px4_sitl gazebo-classic_SHC09
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_SHC09
```
6. SHW09_vtol
```
make px4_sitl gazebo-classic_SHW09_vtol
```
Or, to run the Gazebo (gz) simulation:
```
make px4_sitl gz_SHW09_vtol
```
7. Multirotor with tilt
```
make px4_sitl gazebo-classic_tilt_multirotor
```
Or, to run the Gazebo (gz) tiltrotor simulation:
```
make px4_sitl gz_tiltrotor
```
### Flight with pixhawk

Taking pixhawk 4 as an example, the upload command is:

```
make px4_fmu-v5 upload
```
Other versions are similar, please refer to the official website for more details.

### DuctedFan4 HITL (Hardware-in-the-Loop)

The following procedure runs DuctedFan4 HITL with Gazebo Classic. For other flight controller boards, refer to the [PX4 HITL documentation](https://docs.px4.io/main/en/simulation/hitl).

#### First-time setup

1. For FMUv5, ensure that `boards/px4/fmu-v5/default.px4board` contains:

   ```text
   CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
   ```

   Build and upload the firmware:

   ```bash
   make px4_fmu-v5_default upload
   ```

2. First calibrate the sensors and radio using any normal airframe. Then select the DF4 HITL airframe from the QGC Parameters page:

   ```text
   SYS_AUTOSTART = 1003
   SYS_HITL      = 1
   ```

   Restart the flight controller after changing these parameters. The graphical airframe selection page in QGC is not required.

3. Build Gazebo Classic:

   ```bash
   DONT_RUN=1 make px4_sitl_default gazebo-classic
   ```

4. Identify the flight controller USB serial port during the first setup. Close QGC, then run the appropriate command both before and after connecting the flight controller.

   macOS:

   ```bash
   ls /dev/tty.usbmodem*
   ```

   Ubuntu/Linux:

   ```bash
   dmesg | grep tty
   ls -l /dev/serial/by-id/
   ```

   The macOS device may be `/dev/tty.usbmodem01`, while Linux commonly uses `/dev/ttyACM0`. Set the detected path in:

   ```text
   Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ductedfan4_hitl/ductedfan4_hitl.sdf
   ```

   ```xml
   <serialDevice>/dev/tty.usbmodem01</serialDevice>
   ```

5. In QGC, disable automatic Pixhawk USB and serial connections, leaving only UDP enabled. After this configuration, QGC can remain open: Gazebo owns the USB serial port and forwards the connection to QGC over UDP port 14550.

6. Set up the Gazebo environment and start DF4 HITL:

   ```bash
   # Run from the repository root
   source Tools/simulation/gazebo-classic/setup_gazebo.bash \
       "$(pwd)" \
       "$(pwd)/build/px4_sitl_default"

   gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_ductedfan4.world
   ```

#### Starting HITL after the first-time setup

1. Connect the flight controller and turn on the transmitter.
2. In a terminal where `setup_gazebo.bash` has already been sourced, run:

   ```bash
   gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_ductedfan4.world
   ```

3. Use QGC with only its UDP connection enabled. In a new terminal, source `setup_gazebo.bash` again before starting Gazebo. If the USB device name changes, update `serialDevice` in the SDF file accordingly.
