# 控制分配与 INDI

本文先说明 PX4 控制分配原有的基本关系，包括从物理模型得到 $BU$，以及通过 $D$ 完成归一化控制分配；随后说明 `df-main` 相对 `main` 新增或调整的分配反馈、INDI、PCA、CA16/17 effectiveness source 和 VTOL 路由。公式采用当前 effectiveness source 的共同设置 `linearization_point=0`。

## 1. 从物理模型到 $BU$

### 1.1 带 trim 的物理模型

先定义物理量：

- $v=[M_x,M_y,M_z,F_x,F_y,F_z]^T$：机体系 FRD 力矩和力，单位为 N·m、N；
- $u$：物理执行器量，电机对应推力 N，舵面对应偏转角 rad；
- $u_{\mathrm{trim}}$：物理执行器 trim；
- $B$：从物理执行器量到力矩和力的控制效应矩阵。

零 trim 时，物理模型为：

$$
v=Bu.
$$

加入 trim 后，allocator 使用相对 trim 的模型：

$$
v=B\left(u-u_{\mathrm{trim}}\right).
$$

这里的 $v$ 表示相对 trim 的模型控制作用；本文只讨论 `linearization_point=0`。

### 1.2 引入执行器单位化矩阵 $U$

令 $u_n$ 和 $u_{n,\mathrm{trim}}$ 分别表示归一化执行器设定值和归一化 trim，$U$ 把归一化执行器量转换成物理执行器量：

$$
u=Uu_n,\qquad
u_{\mathrm{trim}}=Uu_{n,\mathrm{trim}}.
$$

因此：

$$
u-u_{\mathrm{trim}}
=U\left(u_n-u_{n,\mathrm{trim}}\right).
$$

定义 allocator 使用的相对 trim 分配变量：

$$
u_{\mathrm{alloc}}:=u_n-u_{n,\mathrm{trim}}.
$$

代回物理模型：

$$
v=BU\,u_{\mathrm{alloc}}.
$$

PX4 effectiveness 参数中填写的控制效应矩阵就是 $BU$。每一列表示某个归一化执行器相对 trim 增加 1 时产生的力矩和力：

- 电机：`CA_ROTORx_CT` 提供最大推力，$U$ 包含最大推力；
- 舵面：`CA_SV_CSx_TRQ_R/P/Y` 已包含最大舵偏，参数值直接属于 $BU$。

$u_{\mathrm{alloc}}$ 专指分配模型中的相对 trim 变量。INDI 文献中的执行器增量继续记作 $\Delta u$。

[mengchaoheng/control_allocation](https://github.com/mengchaoheng/control_allocation) 中的建模脚本使用了同一关系：

| 文件 | 物理矩阵与 $U$ | 写入 PX4 参数的矩阵 |
|---|---|---|
| [`iris.m`](https://github.com/mengchaoheng/control_allocation/blob/master/iris.m) | `B2`，$U=CT\,I_4$ | `B3=B2*CT`，即 $BU$ |
| [`df4.m`](https://github.com/mengchaoheng/control_allocation/blob/master/df4.m) | `B`，$U=(20^\circ)I_4$ | `B*ulim`，即 $BU$ |
| [`df6.m`](https://github.com/mengchaoheng/control_allocation/blob/master/df6.m) | `B`，$U=(20^\circ)I_6$ | `B*ulim`，即 $BU$ |
| [`SHC09.m`](https://github.com/mengchaoheng/control_allocation/blob/master/SHC09.m) | `B`，$U=(40^\circ)I_6$ | `B*ulim`，即 $BU$ |
| [`SHW09_vtol.m`](https://github.com/mengchaoheng/control_allocation/blob/master/SHW09_vtol.m) | `B`，$U=0.6981\,I_8$ | `B*ulim`，即 $BU$ |

五个脚本都采用 $u_{n,\mathrm{trim}}=0$。在 `linearization_point=0` 时，allocator 内部 `_actuator_trim` 对应 $u_{n,\mathrm{trim}}$，`_control_trim=0`，求解和反馈使用同一个 $u_{\mathrm{alloc}}$。

## 2. 从物理模型到归一化控制分配

由上一节得到参数层的物理模型：

$$
v=BU\,u_{\mathrm{alloc}}.
$$

PX4 再引入行缩放对角阵 $D$，定义归一化控制量：

$$
v_{\mathrm{norm}}:=Dv.
$$

代入物理模型后得到完整关系：

$$
v_{\mathrm{norm}}
=D\,v
=D(BU)\,u_{\mathrm{alloc}}.
$$

进入 allocator 时，失效执行器列处理和弱行处理形成 $(BU)_{\mathrm{solver}}$。PX4 对该矩阵求 pseudo-inverse，并依据 mixer 各列计算 $D$。各分配算法共同跟踪的关系为：

$$
D(BU)_{\mathrm{solver}}\,u_{\mathrm{alloc}}
\approx v_{\mathrm{norm,sp}}.
$$

这就是本文所称的归一化控制分配：执行器变量采用归一化范围，控制器和分配器通过归一化控制设定值 $v_{\mathrm{norm,sp}}$ 连接。

`main` 原有 Pseudo-inverse 和 Sequential desaturation；本分支增加 INV、DP_LPCA、DPscaled_LPCA 和 PCA。各算法分别实现上述关系。内置 PID 直接产生 $v_{\mathrm{norm,sp}}$，其控制增益已经吸收相应尺度。角速度 INDI 产生物理力矩 $\tau_c$，随后使用 torque 子块 $D_\tau$ 接入同一个分配接口：

$$
\tau_{\mathrm{norm},c}=D_\tau\tau_c.
$$

关闭 R/P/Y normalization 的 effectiveness source 取 $D_\tau=I_3$。

## 3. 分配反馈及其在 INDI 中的使用

相对 `main`，本分支增加逐 matrix 的 `AllocationValue`，在每次完成分配后发布模型预测控制作用，作为 INDI 的物理反馈。

### 3.1 每个矩阵的反馈

`allocation_value` 的 uORB instance 与 allocation matrix 一一对应：

| topic instance | allocation matrix |
|---|---|
| instance 0 | matrix 0 |
| instance 1 | matrix 1 |

allocator 分别保存求解和反馈使用的矩阵：

- $(BU)_{\mathrm{solver}}$：经过失效执行器列处理和弱行处理，供求解器使用；
- $(BU)_{\mathrm{feedback}}$：经过失效执行器列处理并保留物理交叉效应，供反馈使用。

每个 matrix $i$ 的反馈为：

$$
v_{\mathrm{fb},i}
=\operatorname{LPF}\!\left((BU)_{\mathrm{feedback},i}
u_{\mathrm{alloc,final},i}\right).
$$

$u_{\mathrm{alloc,final}}$ 的起点是当前分配算法求得的执行器结果。该结果随后依次经过 auxiliary control、`updateSetpoint()`、停转电机处理、slew limit 和 clipping，形成用于反馈计算的最终执行器设定值。因此 `allocated_wrench` 表示最终分配及后处理结果对应的模型预测控制作用；当 $BU$ 采用物理单位时，它的单位为 N·m、N。

`AllocationValue` 的主要内容为：

| 字段 | 含义 |
|---|---|
| `allocated_wrench` | 滤波后的 $v_{\mathrm{fb}}$ |
| `control_allocation_scale` | $D$ 的六个对角元素 |
| `effectiveness_bu` | $(BU)_{\mathrm{feedback}}$ |
| `u` | $u_{\mathrm{alloc,final}}$ |
| `feedback_axes_mask` | solver 保留的控制轴 |
| solver status、fallback、rank、rho、timing | LPCA/PCA 诊断 |

allocator 的基础归一化分配残差为：

$$
r_{\mathrm{norm}}=v_{\mathrm{norm,sp}}
-D(BU)_{\mathrm{solver}}\,u_{\mathrm{alloc,final}}.
$$

`ControlAllocatorStatus.unallocated_*` 发布该类信息；Tiltrotor、MC Tilt 和 Helicopter 会把特定轴改为 $-1/0/1$ 饱和方向标志。

### 3.2 角速度 INDI

本分支在 `mc_rate_control` 中通过 `MC_INDI_RATE_EN` 启用角速度 INDI，并使用上一节给出的物理力矩反馈：

$$
\alpha_c=K_p(\omega_{\mathrm{sp}}-\omega),
$$

$$
\tau_c=\tau_0+J(\alpha_c-\alpha_0),
$$

$$
\tau_{\mathrm{norm},c}=D_\tau\tau_c.
$$

$\alpha_c$ 是角速度误差产生的角加速度指令，$\alpha_0$ 是测得的机体角加速度。$\tau_0$ 来自目标 matrix 的 `allocated_wrench`，$D_\tau$ 来自同一 instance 的 `control_allocation_scale`。当前 instance 映射为：

```text
CA_AIRFRAME 16 or 17 -> instance 1
all other values     -> instance 0
```

反馈时效、三轴 mask、数值、惯量和传感器满足运行条件时使用 INDI；其余周期使用 PID。`rate_ctrl_status.indi_active` 记录本周期的实际选择。

### 3.3 加速度 INDI

本分支在 `mc_pos_control` 中通过 `MPC_INDI_ACC_EN` 启用加速度 INDI，并读取 matrix 0 的物理力反馈：

$$
F_0^n=R_b^nF_{\mathrm{fb},0}^b,
$$

$$
F_c^n=F_0^n+m(a_c^n-a_0^n).
$$

$a_c^n$ 是位置/速度控制器产生的加速度指令，$a_0^n$ 是测得的加速度。$F_c^n/m$ 是推力产生的加速度；接回 PX4 时恢复重力项：

$$
a_{\mathrm{map}}^n=\frac{F_c^n}{m}+g e_z,
\qquad e_z=[0,0,1]^T.
$$

$a_{\mathrm{map}}^n$ 随后进入原有 hover-thrust mapping、姿态生成和推力限制流程。

`vehicle_local_position_setpoint.acc_indi_active` 记录本周期是否真正执行了上述 INDI 映射。参数已经开启但车辆尚未起飞、反馈过期或数值无效时，该字段为 `false`。

### 3.4 物理反馈模型的适用前提

当前 INDI 反馈实现面向 `linearization_point=0` 的线性物理模型：目标 matrix 的非零行必须是单位为 N·m、N 的 $BU$，此时最终执行器设定值对应的模型反馈为：

$$
v_0=(BU)\left(u_{n,\mathrm{final}}-u_{n,\mathrm{trim}}\right).
$$

部分内置 effectiveness source 使用相对系数、力矩方向符号或 `updateSetpoint()` 中的非线性映射。例如 Tiltrotor 的 differential tilt、MC Tilt 和 Helicopter；这些分配模型目前不能提供上述物理反馈，也不能仅靠修改 INDI 增益获得正确结果。该限制来自控制分配模型，而不是 INDI 控制律。effectiveness source 将来提供物理线性矩阵，或提供与状态相关的物理前向模型及 Jacobian 后，可以继续使用相同的 INDI 公式和反馈链路。

软件保留时效、有限值和控制轴检查，但不根据数值猜测矩阵是相对量还是物理量。启用 `MC_INDI_RATE_EN` 或 `MPC_INDI_ACC_EN` 表示用户确认目标 matrix 满足物理模型要求。

## 4. PCA 优先级分配

上述角速度 INDI 输出还为本分支新增的 PCA 提供优先级输入。归一化 torque 拆分为：

$$
\tau_{\mathrm{norm},c}=\tau_{\mathrm{norm,indi\ feedback}}
+\tau_{\mathrm{norm,rate\ error}}.
$$

消息字段对应为：

- `xyz_indi_feedback`：higher priority；
- `xyz_rate_error_feedback`：lower priority。

PID 周期把完整 torque 放入 lower 分量。PCA 在 active rows 精确为 R/P/Y、执行器数为 4 至 9 且满行秩时使用该拆分；DP_LPCA 和 DPscaled_LPCA 当前支持 3 或 4 行、4 至 9 个执行器。solver 状态和 fallback 记录每次实际路径。

## 5. VTOL 力矩路由

对 VTOL，角速度 INDI 能否产生控制作用还取决于 virtual topic 到真实 instance 的路由。

### 5.1 虚拟 topic 与真实 instance

VTOL 控制器先发布虚拟 topic：

```text
mc_rate_control -> vehicle_torque_setpoint_virtual_mc
fw_rate_control -> vehicle_torque_setpoint_virtual_fw
```

`vtol_att_control` 再按机型和飞行阶段生成真实 `vehicle_torque_setpoint` instance 0/1，allocator 将它们分别送入 matrix 0/1。

### 5.2 本分支相对 `main` 的 Tailsitter 改动

matrix 0 的控制器来源与 `main` 保持一致：MC/Transition 使用 `virtual_mc`；FW 继续按原逻辑使用 `virtual_fw` 生成可选 differential torque，thrust 也来自 `virtual_fw`。本分支通过 `copy_torque_setpoint()` 同时复制 `xyz`、`xyz_indi_feedback` 和 `xyz_rate_error_feedback`；matrix 1 使用同一 helper 复制所选 surface source，时间戳仍在调用处设置。

实际路由变化位于 `// Control surfaces`：

```cpp
surface_torque_setpoint =
    CA_AIRFRAME == 17 ? virtual_mc : virtual_fw;
```

因此默认 Tailsitter 继续把 `virtual_fw` 送入 surface matrix 1；CA17 选择 `virtual_mc` 作为 surface torque source，MC 阶段同时受 `VT_ELEV_MC_LOCK` 控制。`standard.cpp` 和 `tiltrotor.cpp` 保持 `main` 的路由。

### 5.3 默认 Tailsitter 与 CA17

CA4 和 CA17 都采用 instance 0 → rotor matrix 0、instance 1 → surface matrix 1：

| 阶段 | CA4 matrix 0 | CA4 matrix 1 | CA17 matrix 0 | CA17 matrix 1 |
|---|---|---|---|---|
| MC | `virtual_mc` | lock 时为零，其余为 `virtual_fw` | `virtual_mc` | lock 时为零，其余为 `virtual_mc` |
| Transition | `virtual_mc` | `virtual_fw` | `virtual_mc` | `virtual_mc` |
| FW | `virtual_fw` 可选 differential torque | `virtual_fw` | 与 CA4 相同 | 最新的 `virtual_mc` |

`VT_ELEV_MC_LOCK` 默认值为 1；当前 SHW09 设置为 0。SHW09 的 rotor matrix 0 三轴 torque rows 为零，因此全部 MC torque 由 surface matrix 1 产生，`mc_rate_control` 对应读取 instance 1 的反馈和 $D_{\tau,1}$。

Standard CA2 和 Tiltrotor CA3 采用与 CA4 相同的控制器选择：MC/Transition 使用 `virtual_mc`，FW 使用 `virtual_fw`；CA2/CA3 在 Transition 阶段还会对已经归一化的 `virtual_mc` 逐轴乘 MC 权重。

FW 阶段的 Tailsitter 调度由 `virtual_fw` 更新触发，CA17 在该周期读取最新的 `virtual_mc`。

## 6. INDI 适用范围

角速度 INDI 沿用 `mc_rate_control` 的输出路由；加速度 INDI 沿用 `mc_pos_control` 的输出路由。适用性取决于当前阶段使用的控制器，以及目标 matrix 是否采用第 3.4 节所述的物理 $BU$。

### 6.1 已配置并验证

- Iris：`10015_gazebo-classic_iris`，以及继承其配置的 `22016_gz_iris`；
- DF4、DF6、SHC09；
- SHW09 VTOL：角速度 INDI 支持 MC、Transition、FW，加速度 INDI 支持 MC、Transition；
- CA2 Standard VTOL：`1040_gazebo-classic_standard_vtol`，MC 阶段支持角速度和加速度 INDI；
- CA4 Tailsitter：`1045_gazebo-classic_quadtailsitter`，MC 阶段支持角速度和加速度 INDI。

以上配置已经填写物理 $BU$、质量和惯量。通过 `MC_INDI_RATE_EN` 和 `MPC_INDI_ACC_EN` 分别启用角速度和加速度 INDI。

### 6.2 配置物理效应矩阵后可用

角速度 INDI 的使用条件为：

- 当前阶段实际使用 `mc_rate_control`；
- 目标 matrix 的三轴力矩行为物理 $BU$，单位为 N·m；
- `MC_J_*` 与机体惯量一致。

加速度 INDI 的使用条件为：

- 当前阶段实际使用 `mc_pos_control`；
- matrix 0 的三轴力行为物理 $BU$，单位为 N；
- `MPC_MASS` 与整机质量一致。

满足这些条件后，普通多旋翼、其他 CA2/CA4 VTOL 的 MC 阶段、其他 CA17 DF Tailsitter，以及补齐物理参数后的 DF2、Ducted Fan Mini 均可使用现有 INDI 链路。$D$ 由目标 matrix 的归一化控制分配自动提供。

### 6.3 当前不适用

- CA10/11/12 Helicopter：力矩非线性映射位于 `updateSetpoint()`，allocation matrix 没有可供 INDI 使用的三轴力矩反馈；
- CA3 Tiltrotor 和其他 active-tilt 机型：tilt torque 随当前电机推力和倾转角变化，现有相对力矩列不是物理 N·m；
- CA2/CA3/CA4 VTOL 的 FW 阶段：虽然 `mc_rate_control` 模块已经启动，但实际力矩路径使用 `fw_rate_control`。CA17 SHW09 是当前例外。

这些路径需要先使 effectiveness 能描述目标实例当前实际产生的物理力和力矩，或调整控制器路由；仅修改 INDI 参数不能建立正确反馈。

## 7. 日志确认

日志验证建议同时查看：

- `vehicle_torque_setpoint` instance 0/1：allocator 收到的归一化 torque；
- `allocation_value` instance 0/1：$(BU)_{\mathrm{feedback}}$、$u_{\mathrm{alloc,final}}$、$D$ 和模型反馈；
- `control_allocator_status` instance 0/1：分配状态、饱和和运行时间；
- `rate_ctrl_status.indi_active`：角速度环每周期的 PID/INDI 实际选择；
- `vehicle_local_position_setpoint.acc_indi_active`：加速度环每周期是否真正执行 INDI 映射；
- `vehicle_angular_velocity`、`actuator_motors`、`actuator_servos`：系统响应和最终执行器命令。

## 8. 相关代码

- [ControlAllocator.cpp](src/modules/control_allocator/ControlAllocator.cpp)
- [ControlAllocation.hpp](src/lib/control_allocation/control_allocation/ControlAllocation.hpp)
- [AllocationValue.msg](msg/AllocationValue.msg)
- [MulticopterRateControl.cpp](src/modules/mc_rate_control/MulticopterRateControl.cpp)
- [IndiControl.cpp](src/modules/mc_rate_control/IndiControl/IndiControl.cpp)
- [MulticopterPositionControl.cpp](src/modules/mc_pos_control/MulticopterPositionControl.cpp)
- [tailsitter.cpp](src/modules/vtol_att_control/tailsitter.cpp)
- [standard.cpp](src/modules/vtol_att_control/standard.cpp)
- [tiltrotor.cpp](src/modules/vtol_att_control/tiltrotor.cpp)
