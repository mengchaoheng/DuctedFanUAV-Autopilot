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

## Design Summary
The implementation deliberately separates allocation, feedback, and INDI:

* INV/DP_LPCA/DPscaled_LPCA are generic. PCA is available only when angular-rate INDI supplies its priority split to a torque-only allocation matrix.
* Every active allocation matrix publishes [AllocationValue](msg/AllocationValue.msg) after a real allocation update. It contains the configured `B*U`, normalized actuator result, per-matrix `D`, and filtered allocated wrench.
* The INDI laws are not tied to a vehicle type. Rate INDI is enabled by `MC_USE_INDI`, acceleration INDI by `MPC_USE_ACC_INDI`, and both require recent finite feedback on the axes they use. Only the final MC output/feedback mapper uses `CA_AIRFRAME` to select the corresponding allocation instance. Enabling either controller means the user accepts responsibility for the physical `B*U` contract; rate INDI additionally requires the selected effectiveness source to provide the corresponding finite normalization scale `D`.

This keeps aircraft-specific meaning out of the allocation algorithms. The controller and effectiveness backend decide what the setpoints and matrices mean; the allocator only solves the bounded allocation problem.

### Allocation Methods
PX4 upstream provides pseudo-inverse, sequential desaturation, and `AUTO`. This workspace adds `INV`, `DP_LPCA`, `DPscaled_LPCA`, and `PCA` to `CA_METHOD`.

`CA_METHOD` applies to every allocation instance. If `CA_METHOD=AUTO`, the selected `CA_AIRFRAME` backend supplies its desired method per instance. Otherwise, each instance uses the same selected method.

The LPCA/PCA adapter works on the normalized PX4 allocation contract:

```text
v = B u
u = U u_norm
v = (B U) u_norm
v_norm = D v = D (B U) u_norm = B_norm u_norm
```

Here `v=[tau;F]` is a physical wrench in `[N m, N]`, `u` contains physical rotor thrust `[N]` or surface deflection `[rad]`, and `U` maps normalized actuator commands into those units. Airframe parameters contain `B*U` directly: rotor `CT` is maximum thrust and surface torque parameters already include maximum deflection. `D = diag(_control_allocation_scale)` maps the physical wrench to PX4's normalized control coordinates.

Inside the allocator, `u_norm` and the published wrench use actuator delta relative to allocator trim. The calibrated DF profiles use zero trim, so this is exactly the equation above; a future nonzero-trim model must define `B` about that same operating point.

The LPCA/PCA path builds:

```text
B_norm = D (B U)
find u_norm within actuator bounds so B_norm u_norm tracks v_norm_cmd
```

Current LPCA/PCA applicability:

* active rows: 2 to 6
* actuators: 2 to 16
* actuators must be at least active rows
* active rows must be full-row-rank

Fallback behavior:

* `solver_status = unavailable`: the problem is unsuitable, such as a missing PCA priority split, rank deficiency, fewer than two active rows, more active rows than actuators, or unsupported dimensions.
* `solver_status = failed`: LPCA/PCA dispatch or solver failed for the bounded problem.
* `fallback = 1`: the allocator then runs the internal `INV` path and still produces actuator commands.

PCA adds high/low priority semantics. The solver interface is generic, but this workspace currently defines only the three-dimensional torque split generated by angular-rate INDI:

```text
higher = INDI torque feedback
lower  = rate-error torque feedback
```

[ControlAllocator.cpp](src/modules/control_allocator/ControlAllocator.cpp) only passes that split when angular-rate INDI is active and the target allocation matrix is torque-only. PCA runs only with that valid split; otherwise it is unavailable for the cycle and falls back to `INV`.

### Allocation Feedback
[ControlAllocatorStatus](msg/ControlAllocatorStatus.msg) is public allocator status for all airframes. It reports allocation success, unallocated torque/thrust, saturation state, and allocation runtime:

```text
allocation_running_time
allocation_running_time_avg
allocation_running_time_samples
```

[AllocationValue](msg/AllocationValue.msg) is per-matrix physical feedback and solver diagnostics. Every configured matrix publishes it after a real allocation update, independently of airframe type and independently of whether INDI is enabled. A backup scheduler run never republishes an old actuator result with a fresh timestamp.

For matrix `i`, the allocator reports:

```text
w_i = (B_i U_i) (u_norm_i - trim_i)
```

The message contains the fallback and solver diagnostics, requested physical wrench, physical `B*U`, normalized actuator delta, row scale `D_i`, and filtered allocated wrench `w_i`. `num_actuators` identifies the valid prefix of `u` and the valid columns of the fixed-size `B*U` array; the wrench dimension is always six and therefore has no separate dimension field. Total allocator runtime remains in `ControlAllocatorStatus` instead of being duplicated here. The wrench is formed after effectiveness updates, auxiliary controls, stopped-motor handling, slew limiting, and clipping. Filtering is applied to the completed wrench; the allocator does not simulate individual actuator dynamics.

Only an explicitly stopped motor's NaN command is converted back to physical zero thrust. Any other non-finite actuator value invalidates only the wrench rows to which that actuator has nonzero effectiveness. The reported physical `B*U` is captured after failed actuator columns are removed but before the legacy weak-row suppression, while `feedback_axes_mask` describes the rows actually retained by the solver. Thus feedback keeps real cross-effects without claiming control authority on a suppressed row, and publication is no longer coupled to the former DF-only gate.

The upstream allocator still suppresses a row when every configured coefficient on that row has magnitude at most `0.05`. This existing solver rule also limits physical `B*U` models for very small aircraft; it is independent of INDI and is not changed by this integration.

There is no vehicle-level aggregate message in the control path. The MC output mapper selects the allocation instance used by the configured effectiveness source, and angular-rate INDI reads feedback and `D` from that same instance. The calibrated profiles include Iris 10015 and DF airframes 22002, 22003, 22005, and 22006 (plus hardware 22005). Other MC airframes can use INDI once their configured `B*U`, mass, and inertia are physical. PX4 checks only runtime availability and finite values; the user is responsible for the model.

When an effectiveness source disables R/P/Y normalization, its torque scale is simply `D=I`. That is valid for physical INDI too, but the existing legacy `CA_AIRFRAME=9` DF2/mini coefficients are normalized shape matrices rather than calibrated N/N-m models.

### INDI Conditions And Feedback
Angular-rate INDI lives in [mc_rate_control](src/modules/mc_rate_control) and is enabled by `MC_USE_INDI`. The module keeps the standard PX4 startup (`mc_rate_control start`, or `start vtol`). Only its final output mapping depends on `CA_AIRFRAME`: normal MC output uses torque instance 0, CA16 uses torque instance 1, and CA17 keeps the `virtual_mc` path for `vtol_att_control` to route to instance 1. The PID and INDI calculations do not contain airframe-specific branches.

INDI uses a recent `AllocationValue` instance with all three torque axes, finite allocated torque, and finite positive torque `D`. If that feedback, the sensors, or the INDI gains/inertia are invalid, the same cycle uses the original PID controller.

Its feedback extraction is:

```text
tau_0 = filtered((B*U)_torque * u_norm)
alpha_c = Kp * (rate_sp - rate)
indi_fb = tau_0 - J * angular_accel
error_fb = J * alpha_c
torque_phys = indi_fb + error_fb
torque_norm = D_torque * torque_phys
```

Both PID and INDI publish the existing normalized `vehicle_torque_setpoint`; only the INDI branch needs the selected matrix's `D` conversion. The message also carries a same-cycle PCA decomposition whose two components always sum to `xyz`: INDI publishes `indi_fb` as the higher-priority component and `error_fb` as the lower-priority component, while PID fallback publishes zero and the complete setpoint respectively. Selecting `CA_METHOD=PCA` runs prioritized allocation only for a matrix whose active rows are exactly `Mx/My/Mz`; other matrices fall back to INV. PCA configurations are expected to enable `MC_USE_INDI`.

Acceleration INDI lives in [mc_pos_control](src/modules/mc_pos_control) and is enabled by `MPC_USE_ACC_INDI` while flying. It reads the recent, finite matrix-0 force vector because PX4 places motors/thrust in matrix 0.

Its feedback extraction is:

```text
force_0_body = filtered((B*U)_force * u_norm)
force_0_ned = R_to_ned * force_0_body
force_c_ned = force_0_ned + MPC_MASS * (acc_sp - acc_meas)
thrust_acc_sp = force_c_ned / MPC_MASS
acc_sp_indi = thrust_acc_sp + g * e_z
thrust_norm = (MPC_THR_HOVER / g) * thrust_acc_sp
```

The physical force is divided by mass and converted into the acceleration setpoint expected by PX4. From there the original hover-thrust mapping, `MPC_ACC_DECOUPLE` behavior, attitude generation, thrust limiting, and anti-windup path are reused. Acceleration INDI does not use allocation `D`; the built-in acceleration-to-normalized-thrust mapping remains the output mapping.

### Ducted-Fan Effectiveness
`CA_AIRFRAME=16` uses [ActuatorEffectivenessDuctedFan.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFan.cpp). It creates two allocation matrices:

* instance 0: motor force allocation, with propeller torque disabled
* instance 1: control-surface torque allocation

The MC output mapper publishes torque to instance 1 while thrust remains on instance 0. Keeping matrix 1 torque-only also lets PCA apply the existing `INDI torque > rate-error torque` split.

`CA_AIRFRAME=17` uses [ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp](src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessDuctedFanTailsitterVTOL.cpp). It inherits PX4's tailsitter two-instance structure. `vtol_att_control` performs the existing MC/FW output routing, while the effectiveness source adjusts the surface matrix by flight phase.

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
