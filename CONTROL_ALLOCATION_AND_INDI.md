# 控制分配与 INDI

本文按代码来源组织内容：

- 第 1、2 节解析 PX4 内置的 $BU$ effectiveness、trim、归一化控制和 allocation matrix 机制；
- 第 3、4 节说明本分支新增的 INV、LPCA、PCA、诊断、运行时间和物理反馈；
- 第 5 节先解析 PX4 内置 VTOL 路由，再说明 CA16、CA17 的项目扩展；
- 第 6、7 节说明 rate INDI 与 acceleration INDI；
- 第 8 节记录 22002 实飞日志、时延分析和本次 acceleration INDI 修正；
- 第 9、10 节说明消息日志和硬件配置。

全文描述当前代码事实、接口契约和设计选择。主题集中在 `control_allocator`、`mc_rate_control`、`mc_pos_control` 及其 VTOL 路由；`mc_att_control` 与 `MC_ATT_ERR_MODE` 属于姿态控制专题。

## 1. PX4 内置 effectiveness、trim 与执行器变量

### 1.1 Effectiveness matrix 的内置含义

PX4 control allocation 使用六维控制向量：

$$
c=[M_x,M_y,M_z,F_x,F_y,F_z]^T,
$$

坐标系为机体系 FRD。定义 $u$ 为执行器物理量或机型建模量，$u_n$ 为 PX4 归一化执行器输入，$U$ 为二者之间的比例矩阵：

$$
u=Uu_n.
$$

定义 $B$ 为执行器物理量或机型建模量到控制作用的矩阵：

$$
c=B(u-u_{\mathrm{trim}}).
$$

代入归一化执行器关系后：

$$
c=BU(u_n-u_{n,\mathrm{trim}}).
$$

因此 PX4 effectiveness source 构造并由用户参数填写的矩阵就是：

$$
B_s=BU.
$$

矩阵列对应归一化执行器输入，矩阵行对应六个控制轴。在线性分配阶段：

$$
c_{\mathrm{alloc}}=B_s\,\Delta u_n.
$$

$\Delta u_n$ 表示执行器设定值相对 allocator trim 的增量。$B_s=BU$ 描述 PX4 内置接口的坐标关系，矩阵数值的单位由 effectiveness source 与机型参数共同定义。PX4 原有机型广泛采用相对控制单位；具备物理标定的机型在同一 $BU$ 接口中使用 N·m、N。

电机、舵面、倾转机构等 source 都通过这个接口描述控制作用。`CA_ROTORx_CT`、`CA_ROTORx_KM`、旋翼轴与位置共同形成 rotor 列；`CA_SV_CSx_TRQ_R/P/Y` 形成 control-surface 列。

### 1.2 Trim 与 linearization point

`ControlAllocation::setEffectivenessMatrix()` 对 linearization point $u_{n,l}$ 执行上下界裁剪，然后建立：

$$
u_{n,\mathrm{trim,internal}}
=u_{n,\mathrm{trim,param}}+u_{n,l},
$$

$$
c_{\mathrm{trim}}=B_su_{n,l}.
$$

分配器围绕内部 trim 计算 actuator delta：

$$
\Delta u_n=u_{n,sp}-u_{n,\mathrm{trim,internal}},
$$

并以 $c_{sp}-c_{\mathrm{trim}}$ 作为待分配控制量。`linearization_point=0` 时：

$$
u_{n,\mathrm{trim,internal}}=u_{n,\mathrm{trim,param}},
\qquad c_{\mathrm{trim}}=0.
$$

### 1.3 从算法输出到 actuator topic

PX4 内置 control allocation 周期按以下顺序处理执行器设定值：

1. allocation algorithm 计算归一化 actuator setpoint；
2. effectiveness source 叠加 flap、spoiler 等 auxiliary controls；
3. effectiveness source 通过 `updateSetpoint()` 应用机型状态逻辑；
4. stopped-motor mask 把相应电机通道写成 NaN；
5. slew-rate limiter 处理执行器变化率；
6. actuator limits 完成最终裁剪；
7. `actuator_motors` 与 `actuator_servos` 发布最终输出。

NaN 电机通道在 PX4 actuator topic 中表达停转命令，电机驱动侧据此进入 disarmed output。

## 2. PX4 内置归一化控制与多矩阵框架

### 2.1 控制轴归一化

Pseudo-inverse 首先由 $B_s$ 计算原始伪逆 $B_s^+$，再从各控制轴对应的伪逆列计算 scale。令：

$$
D=\operatorname{diag}(d_R,d_P,d_Y,d_{F_x},d_{F_y},d_{F_z}).
$$

PX4 对伪逆列除以相应 scale，等价于使用：

$$
(DB_s)\Delta u_n\approx c_{\mathrm{norm},sp},
$$

其中：

$$
c_{\mathrm{norm}}=Dc_s.
$$

因此控制器输出与 allocator 输入采用归一化控制量，执行器变量采用归一化 actuator range。内置 rate PID 的增益已经包含这个接口尺度。物理力矩控制器接入同一 torque 接口时使用 torque 子块：

$$
\tau_{\mathrm{norm}}=D_\tau\tau.
$$

effectiveness source 通过 `getNormalizeRPY()` 选择 R/P/Y scale 的计算方式。关闭 R/P/Y normalization 时 $D_\tau=I_3$。

常用配置采用 `metric_allocation=false`，并由 effectiveness source 为相应 matrix 开启 R/P/Y normalization。此时 $D$ 吸收每个控制轴的统一行尺度，effectiveness 可采用相对控制单位；各执行器列之间的比例、符号和几何关系继续决定分配结果。启用 INDI 时，机型填写的 $B*U$ 需要与对应模型的理论值足够接近，并采用第 4.1 节的 SI 物理单位。

### 2.2 内置分配方法与 Automatic

PX4 主线提供两种分配方法：

- Pseudo-inverse：使用归一化伪逆并裁剪 actuator output；
- Sequential desaturation：在伪逆基础上按机型定义的 desaturation 顺序处理饱和。

`CA_METHOD=Automatic` 会逐 matrix 查询 effectiveness source 的默认方法。Multirotor、MC Tilt、部分旋翼 source 与 UUV 通常选择 Sequential desaturation；常规 VTOL source 通常为 matrix 0 选择 Sequential desaturation，为 matrix 1 选择 Pseudo-inverse。

### 2.3 Active rows、弱控制行与 allocation matrix

Control allocator 支持多个 allocation matrix。每个 matrix 拥有独立的 effectiveness、trim、actuator limits、算法实例和 `control_allocator_status` topic instance。

PX4 内置 weak-row 处理会扫描每一行。某行所有 effectiveness 元素的绝对值均低于 `0.05` 时，solver matrix 将该行清零，分配器把控制权集中到主要控制轴。Motor failure handling 也会把对应执行器列清零。

`control_allocator_status` 的原有字段记录 normalized control 域中的 allocated control、unallocated torque/thrust、setpoint achieved、saturation 和 motor failure 信息。

## 3. 本分支新增的分配算法

`CA_METHOD` 当前包含：

| 值 | 方法 | 运行条件 | 运行路径 |
|---:|---|---|---|
| 0 | Pseudo-inverse | effectiveness source 已完成配置 | PX4 原有实现 |
| 1 | Sequential desaturation | effectiveness source 已完成配置 | PX4 原有实现 |
| 2 | Automatic | effectiveness source 提供逐 matrix 默认方法 | 创建阶段选择方法 |
| 3 | INV | 任意已配置 active-row matrix | 归一化伪逆与 actuator-delta 限幅 |
| 4 | DP_LPCA | LPCA 结构条件成立 | LP 求解，异常路径转入 INV |
| 5 | DPscaled_LPCA | LPCA 结构条件成立 | scaled LP 求解，异常路径转入 INV |
| 6 | PCA | LPCA 结构条件、显式 priority split、R/P/Y active axes 同时成立 | prioritized LP 求解，异常路径转入 INV |

INV、DP_LPCA、DPscaled_LPCA、PCA 通过 `CA_METHOD=3～6` 显式选择。CA16、CA17 的 matrix 0 是单电机 force matrix，当前代码为该 matrix 选择 INV，并把用户选择的 LPCA/PCA 方法保留在 matrix 1。

### 3.1 INV

INV 在归一化 solver matrix 的 active rows 上计算伪逆：

$$
\Delta u_n=(DB_s)^+c_{\mathrm{norm},sp}.
$$

每个 actuator delta 随后限制到相对 trim 的上下界。数值异常路径把 actuator delta 置零，因此输出回到 trim。

### 3.2 DP_LPCA 与 DPscaled_LPCA

LP solver 的结构范围为：

1. active-row matrix 满行秩；
2. active rows 数量为 3 或 4；
3. actuator 数量为 4～9；
4. actuator 数量大于等于 active rows 数量。

DP_LPCA 求解方向保持分配，DPscaled_LPCA 求解带 actuator-bound scaling 的方向保持分配。零控制指令直接产生零 actuator delta。以下事件触发 INV 路径：

- active-row matrix 出现秩缺失；
- rows 或 actuator 数量落在模板范围外；
- LP 初始可行解、迭代或 restoring 返回错误；
- solver output 出现 NaN 或 Inf。

### 3.3 PCA priority split

PCA 的使用前提由两部分组成：

1. active rows 恰好为 R/P/Y；
2. 当前 torque setpoint 携带显式 priority split。

`VehicleTorqueSetpoint` 中：

- `xyz` 保存完整 normalized torque setpoint；
- `xyz_indi_feedback` 保存 higher-priority normalized torque；
- `xyz_indi_feedback_valid` 标记本周期已建立 priority split。

PCA 构造：

$$
\mathrm{higher}=\texttt{xyz\_indi\_feedback},
$$

$$
\mathrm{lower}=\texttt{xyz}-\texttt{xyz\_indi\_feedback}.
$$

rate INDI active 周期填写 higher 与 valid。higher 恰好为零时，valid 仍然准确表达 priority split。PID 周期与其他 torque publisher 保持 valid 的默认值，PCA 随后选择 INV 路径。active axes、matrix rank、template range 或 solver result 进入异常状态时也选择 INV 路径。

`solver_err=4` 表示 PCA axes 前提落在范围外，`solver_err=5` 表示 priority split 缺失。

### 3.4 Solver 诊断与运行时间

本分支扩展已有 `ControlAllocatorStatus`，每个 allocation matrix 对应一个 topic instance：

| 字段 | 含义 |
|---|---|
| `fallback` | 当前周期从 LPCA/PCA 转入 INV |
| `solver_status` | solver 跳过、成功、结构条件异常或执行失败 |
| `solver_err` | adapter 前置条件或底层 solver 错误码 |
| `full_row_rank` | active-row matrix 满行秩状态 |
| `solver_rho` | LPCA/PCA 返回的方向或缩放量 |
| `solver_prepare_time` | adapter 在底层 solver 对象构造前的耗时，µs |
| `solver_core_time` | 底层对象构造、求解和 restoring 耗时，µs |
| `solver_post_time` | solver 返回后的检查与复制耗时，µs |
| `allocation_running_time` | 最近一次完整 `allocate()` 耗时，µs |
| `allocation_running_time_avg` | 完整 `allocate()` 的在线平均耗时，µs |
| `allocation_running_time_samples` | 在线平均累计样本数 |

`solver_status=-1` 表达 solver 入口前的结构条件事件，`solver_status=-2` 表达 solver 执行事件。两种状态配合 `fallback=true` 表示当前周期的 actuator output 来自 INV。SITL 计时采用 monotonic wall clock，FMU 计时采用 HRT。

### 3.5 Host test 与板上验证

`ControlAllocationLPCATest` 覆盖：

- 3/4 active rows 与 4～9 actuators 的全部模板组合；
- PCA priority split、zero higher command、priority 缺失；
- 持续饱和下的 actuator bounds；
- rank、dimension 与 PCA axes 的 INV 路径。

Host test 验证数值结果和调度分支。FMUv5 通过目标机型、最大矩阵、持续饱和与 `top once` 验证 work queue 栈高水位；`control_allocator status` 同时显示 solver 状态和 allocation timing。

## 4. 本分支新增的物理分配反馈

### 4.1 $BU$ 的物理单位契约

第 1 节已经给出 PX4 内置关系 $B_s=BU$。本项目为 INDI 机型进一步规定 $BU$ 的控制行使用 SI 物理单位。定义：

- $v=[M_x,M_y,M_z,F_x,F_y,F_z]^T$：机体系 FRD wrench，单位 N·m、N；
- $u$：物理执行器量，电机使用推力 N，舵面使用偏转角 rad；
- $u_n$：归一化执行器量；
- $U$：归一化执行器量到物理执行器量的比例矩阵；
- $B$：物理执行器量到 wrench 的矩阵。

关系为：

$$
u=Uu_n,
$$

$$
v=B(u-u_{\mathrm{trim}})
=BU(u_n-u_{n,\mathrm{trim}}).
$$

具备 INDI 物理反馈的机型把 $BU$ 配置为物理 wrench effectiveness：

$$
B_s=BU\quad [\mathrm{N\,m},\mathrm{N}].
$$

因此每一列表示 normalized actuator delta 增加 1 对应的物理 wrench。电机 `CA_ROTORx_CT` 使用 normalized thrust 1 对应的最大推力 N；舵面 `CA_SV_CSx_TRQ_R/P/Y` 使用 normalized surface delta 1 对应的 N·m，并已经包含最大舵偏。

[mengchaoheng/control_allocation](https://github.com/mengchaoheng/control_allocation) 中的建模脚本采用相同关系：

| 文件 | 物理矩阵与 $U$ | 写入 PX4 参数的矩阵 |
|---|---|---|
| [`iris.m`](https://github.com/mengchaoheng/control_allocation/blob/master/iris.m) | `B2`，$U=CTI_4$ | `B3=B2*CT=BU` |
| [`df4.m`](https://github.com/mengchaoheng/control_allocation/blob/master/df4.m) | `B`，$U=(20^\circ)I_4$ | `B*ulim=BU` |
| [`df6.m`](https://github.com/mengchaoheng/control_allocation/blob/master/df6.m) | `B`，$U=(20^\circ)I_6$ | `B*ulim=BU` |
| [`SHC09.m`](https://github.com/mengchaoheng/control_allocation/blob/master/SHC09.m) | `B`，$U=(40^\circ)I_6$ | `B*ulim=BU` |
| [`SHW09_vtol.m`](https://github.com/mengchaoheng/control_allocation/blob/master/SHW09_vtol.m) | `B`，$U=0.6981I_8$ | `B*ulim=BU` |

这些物理反馈机型采用 `linearization_point=0`，反馈增量为：

$$
\Delta u_n=u_{n,\mathrm{final}}-u_{n,\mathrm{trim}}.
$$

### 4.2 Solver matrix 与 physical matrix

本分支从同一 effectiveness 配置保存两个视图：

- solver matrix：包含 motor-failure column handling 与 PX4 weak-row handling；
- physical matrix：包含相同的 motor-failure column handling，并保留物理交叉效应行。

solver matrix 决定算法主动跟踪的控制轴。physical matrix解释最终执行器设定值对应的模型 wrench：

$$
v_{fb,i}=(BU)_{\mathrm{physical},i}
\left(u_{n,\mathrm{final},i}-u_{n,\mathrm{trim},i}\right).
$$

反馈计算发生在 auxiliary controls、`updateSetpoint()`、stopped-motor handling、slew-rate limiting 和 actuator clipping 之后。

### 4.3 `AllocationValue`

本分支新增单实例高速消息 `AllocationValue`：

| 字段 | 含义 |
|---|---|
| `timestamp` | 反馈发布时间 |
| `timestamp_sample` | 触发本次分配的 torque setpoint 样本时间 |
| `raw_allocated_torque` | 滤波前 body-FRD 物理力矩，N·m |
| `raw_allocated_force` | 滤波前 body-FRD 物理力，N |
| `allocated_torque` | 滤波后 body-FRD 物理力矩，N·m |
| `allocated_force` | 滤波后 body-FRD 物理力，N |
| `torque_setpoint_scale` | $D_\tau$ 的 R/P/Y 对角元素 |

Actuator vectors 继续由 `actuator_motors`、`actuator_servos` 记录；solver 状态和 timing 继续由 `ControlAllocatorStatus` 记录。

### 4.4 NaN 与 stopped motors

PX4 actuator topic 中的 NaN 表达执行器通道状态。典型来源包括：

- actuator topic 的尾部通道；
- effectiveness source 的飞行阶段停转逻辑；
- failure handling 与 failure injection；
- launch lock、tilt disarmed position、helicopter motor disengage 等 `updateSetpoint()` 状态逻辑；
- 参数、控制输入或数值计算传播出的 NaN/Inf。

当前 INDI 布局把 motor channels 放在 matrix 0。stopped-motor mask 标记的 NaN 电机具有零物理推力，反馈计算使用：

$$
\Delta u_n=0-u_{n,\mathrm{trim}}.
$$

其余已配置执行器出现 NaN 时，`AllocationValue` 相应 wrench 进入 NaN 状态，rate 与 position controller 在该周期选择 PX4 原控制路径。

### 4.5 反馈滤波

三轴 torque 使用 `CA_TORQ_CUTOFF`，三轴 force 使用 `CA_FORCE_CUTOFF`。两个二阶低通滤波器组的默认 cutoff 均为 8 Hz。

滤波采样率来自连续两次 `AllocationValue` 的实际发布时间：

1. 首个样本直接发布 raw value；
2. 后续发布间隔限制到 0.1～20 ms；
3. 发布间隔使用 0.25 s 时间常数平滑；
4. 采样率相对变化超过 10% 时重配滤波器；
5. 重配使用当前输出恢复 delay state；
6. effectiveness configuration update 或 100 ms 发布间隔触发状态重建；
7. cutoff 为零时相应轴直通。

采样率更新跟随 allocator 真实运行周期，cutoff 保持机型调参属性。raw 与 filtered 字段支持实机评估噪声衰减和相位滞后。

## 5. VTOL 内置路由与 CA16/17 项目扩展

### 5.1 PX4 内置 VTOL 路由

VTOL 同时运行 MC 和 FW 控制链。两个控制器先发布 virtual setpoint：

```text
mc_rate_control -> vehicle_torque_setpoint_virtual_mc / vehicle_thrust_setpoint_virtual_mc
fw_rate_control -> vehicle_torque_setpoint_virtual_fw / vehicle_thrust_setpoint_virtual_fw
```

`vtol_att_control` 根据机型、模式和过渡状态组装 allocator 使用的 instance 0/1：

$$
\begin{bmatrix}
c_0\\
c_1
\end{bmatrix}
=
\Phi_{\mathrm{VTOL}}(\text{机型},\text{阶段})
\begin{bmatrix}
c_{mc}\\
c_{fw}
\end{bmatrix}.
$$

常规 VTOL 的 matrix 0 主要连接旋翼与电机，matrix 1 主要连接气动舵面。MC、FW 与 transition 阶段使用权重、推力转换和差动推力配置改变两个 matrix 的 setpoint 来源。因此 allocation instance 表达“该 matrix 当前实现的控制作用”。

Tailsitter 的内置 `fill_actuator_outputs()` 在 rotary-wing 与 transition 阶段把 MC thrust/torque 送入 matrix 0，并把 FW surface torque 送入 matrix 1；forward-flight 阶段 matrix 0 接收 FW thrust 与可选 differential torque，matrix 1 接收 FW surface torque。阶段切换后的 50 ms bridge 保持 thrust 连续。

### 5.2 CA0、CA9、CA16、CA17 的当前布局

当前 INDI 反馈布局为：

| `CA_AIRFRAME` | Force feedback | Torque feedback | 控制路由 |
|---:|---|---|---|
| 0 | matrix 0 | matrix 0 | 单 matrix MC |
| 9 | matrix 0 | matrix 0 | 单 matrix Custom |
| 16 | matrix 0 | matrix 1 | 单电机 force 与 control-surface torque 分离 |
| 17 | matrix 0 | matrix 1 | tailsitter force 与 MC control-surface torque 分离 |

CA16 是 MC module：`mc_rate_control` 保持 thrust 在 instance 0，将 instance 0 torque 清零，并把完整 torque setpoint、INDI higher 和 valid 显式发布到 instance 1。Allocator 因而保持通用的 instance 0 / 1 到 matrix 0 / 1 路由。Matrix 0 的单电机 effectiveness 只包含 force，matrix 1 的舵面 effectiveness 只包含 torque。

因此 CA16 每周期交给两个 allocation matrix 的完整目标为：

| | matrix 0 / instance 0 | matrix 1 / instance 1 |
|---|---|---|
| Torque target | $\boldsymbol{0}$ | MC torque setpoint，以及 PCA 使用的 INDI higher 和 valid |
| Force target | MC thrust setpoint | $\boldsymbol{0}$ |

即 matrix 0 target 为 `[0 torque, MC thrust]`，matrix 1 target 为 `[MC torque, 0 thrust]`。CA16 不发布 `vehicle_thrust_setpoint[1]`；allocator 中 matrix 1 的 force 分量保持零值。相应地，日志中的 `vehicle_thrust_setpoint_0` 与 `control_allocator_status_0` 用于检查 force 路径，`vehicle_torque_setpoint_1` 与 `control_allocator_status_1` 用于检查 torque/PCA 路径。

CA17 是 VTOL module：`mc_rate_control` 发布 virtual MC setpoint，tailsitter 路由把 matrix 0 torque 设为零，并在所有 VTOL 阶段将 virtual MC 的完整 torque setpoint、INDI higher 和 valid 作为 instance 1 / matrix 1 的控制目标。Matrix 0 thrust 继续采用 tailsitter 内置分阶段路由。Acceleration INDI 的消费周期属于 multicopter position-control 模式。

CA17 effectiveness 继承内置 tailsitter 两矩阵结构，并增加 flight-phase matrix update：

- hover 阶段把 surface 6/7 的 effectiveness 置零；
- transition 与 forward-flight 阶段把 surface 0～5 的 torque rows 乘 `DF_FW_CS_GAIN`；
- matrix update 由 flight phase 变化触发。

CA9 的 rotor model 启用 propeller torque，使自定义旋翼反扭矩进入 effectiveness 与物理反馈。

## 6. Rate INDI

### 6.1 控制律

Rate INDI 使用：

$$
\alpha_c=K_p(\omega_{sp}-\omega),
$$

$$
\tau_c=\tau_0+J(\alpha_c-\alpha_0).
$$

代码把物理输出分成：

$$
\tau_{rate}=J\alpha_c,
$$

$$
\tau_{feedback}=\tau_0-J\alpha_0.
$$

完整 normalized torque 与 PCA higher component 分别为：

$$
\tau_{norm,c}=D_\tau(\tau_{rate}+\tau_{feedback}),
$$

$$
\tau_{norm,higher}=D_\tau\tau_{feedback}.
$$

Yaw output low-pass 与 battery scaling 作用于最终 torque。higher component 按最终输出与滤波前总输出的逐轴比例同步缩放，使 higher + lower 始终等于发布的 `xyz`。

### 6.2 启用条件与周期选择

`rate_ctrl_status.indi_active` 综合以下层次：

| 层次 | 判断时点 | 条件 |
|---|---|---|
| 参数门 | 启动与 parameter update | `MC_INDI_RATE_EN=1` |
| 机型门 | 启动与 parameter update | `CA_AIRFRAME` 为 0、9、16、17 |
| 模型参数门 | 启动与 parameter update | 三轴 INDI gain 为 finite，`MC_J_X/Y/Z` 为 finite positive |
| 控制模式门 | 每个 gyro 周期 | `flag_control_rates_enabled=true` |
| 反馈门 | 每个 rate-control 周期 | `AllocationValue` 已发布、年龄小于 100 ms、torque 与 scale 为 finite |

原 rate PID 每个 rate-control 周期都执行并更新 integrator，形成 hot standby。INDI feedback gate 成立时，INDI torque 覆盖本周期待发布 torque；反馈 gate 转换时，输出选择在 PID 与 INDI 之间逐周期切换。PID integrator 保持连续，disarmed 与 vehicle type 对应的原版 integral reset 规则持续生效。

CA0/9 使用 `control_allocator_status` instance 0 更新 PID anti-windup；CA16/17 使用 instance 1。`vehicle_torque_setpoint.xyz_indi_feedback_valid` 同时记录 PCA priority split 的实际状态。

## 7. Acceleration INDI

### 7.1 控制律与符号约定

Allocator 发布 body-FRD signed force $F_0^b$。本项目的 force 向量采用 PX4 thrust-force 方向：

$$
F=-T b_z.
$$

Position controller 使用当前 attitude 转换到 NED：

$$
F_0^n=R_b^nF_0^b.
$$

原 position/velocity controller 先生成期望惯性加速度 $a_c^n$，local-position state 提供测得加速度 $a_0^n$。INDI 计算：

$$
F_c^n=F_0^n+m(a_c^n-a_0^n).
$$

代码先保存 allocated thrust acceleration：

$$
a_{T,0}^n=\frac{F_0^n}{m},
$$

再在 `PositionControl::_accelerationControl()` 中建立：

$$
a_{T,c}^n=a_{T,0}^n+(a_c^n-a_0^n).
$$

PX4 `_accelerationControl()` 接收惯性加速度约定，因此代码加入 NED gravity：

$$
a_{map}^n=a_{T,c}^n+g[0,0,1]^T.
$$

随后继续使用 PX4 内置 hover-thrust mapping、attitude generation、tilt limit、vertical priority、horizontal margin 和 thrust limit。

### 7.2 Hover-thrust scale 与 force scale

PX4 原映射的垂向核心关系为：

$$
T_{norm,z}=a_{map,z}\frac{T_h}{g}-T_h.
$$

`PositionControl` 启动时从 `MPC_THR_HOVER` 初始化 $T_h$。普通 PID 模式收到有效
`hover_thrust_estimate` 后调用 `updateHoverThrust()`：它在更新 $T_h$ 的同时修正垂向速度积分器，使参数变化前后的 normalized thrust 尽量连续。

Acceleration INDI active 且反馈有效时采用不同处理：代码通过 `setHoverThrust()` 更新 $T_h$，不向垂向积分器注入补偿。此时速度环 I、D 仍作为普通 PID hot standby 在后台更新，但不进入 INDI 的期望加速度；INDI 使用：

$$
a_c=K_v(v_{sp}-v)+a_{ff}.
$$

位置环仍在上游通过位置误差生成 $v_{sp}$，所以完整串级结构对应论文中的位置误差项、速度误差项和加速度前馈。退出 INDI 后普通 PID 的 I、D 参数和积分状态仍可继续使用。本实现不在模式切换时修改 `MPC_XY_VEL_D_ACC`、`MPC_Z_VEL_D_ACC` 等参数，也不清零积分器；屏蔽发生在每个 INDI 有效控制周期的控制律选择中。

地面阶段使用原 takeoff logic 恢复 `MPC_THR_HOVER`。当前生效的 hover thrust 完成 acceleration 到 normalized thrust 的尺度转换。

Rate INDI 直接向 normalized torque allocator interface 发布控制量，因此 `AllocationValue` 携带 $D_\tau$。Acceleration INDI 先回到 PX4 acceleration interface，再由 hover-thrust mapping 生成 normalized thrust，因此消息字段集中于 torque scale。

### 7.3 启用条件与周期选择

`vehicle_local_position_setpoint.acc_indi_active` 综合以下层次：

| 层次 | 判断时点 | 条件 |
|---|---|---|
| 参数门 | 启动与 parameter update | `MPC_INDI_ACC_EN=1` |
| 机型门 | 启动与 parameter update | `CA_AIRFRAME` 为 0、9、16、17 |
| 质量门 | 启动与 parameter update | `MPC_MASS` 为 finite positive，并缓存 inverse mass |
| 控制模式门 | 每个 local-position 周期 | `flag_multicopter_position_control_enabled=true` |
| Force feedback 门 | 每个 position-control 周期 | `AllocationValue` 已发布、年龄小于 100 ms、force 为 finite |
| Attitude 门 | 每个 position-control 周期 | attitude 已发布、年龄小于 100 ms、body-to-NED 结果为 finite |
| 原控制器输入门 | 每个 position-control 周期 | PositionControl setpoint/state 满足原 `_inputValid()` 契约 |
| INDI 动态门 | `_accelerationControl()` | $a_c^n$ 与 $a_0^n$ 为 finite |

任一 feedback gate 转入异常状态时，`allocated_thrust_acceleration` 保持 NaN，`_accelerationControl()` 直接执行 PX4 原 acceleration-to-thrust 映射。PositionControl 的 last-valid-setpoint 与 failsafe 流程继续承担原输入异常处理。

## 8. 22002 实飞日志与 acceleration INDI 修正

### 8.1 日志范围与飞行现象

分析范围为 `2026-8-1/log_71` 至 `log_82` 中 `SYS_AUTOSTART=22002` 的 ductedfan4 实飞日志。`log_73_2026-8-1-18-46-58.ulg` 的 `SYS_AUTOSTART=1003`，不属于本机型，已排除；同编号的 `log_73_2026-8-1-20-14-28.ulg` 属于 22002。

Acceleration INDI 使用：

$$
a_{out}
=\frac{F_{alloc}}{MPC\_MASS}
+(a_c-a_0)+g e_z.
$$

其中 $F_{alloc}$ 不是传感器实测推力，而是最终 actuator command 经过物理 allocation matrix 回算得到的模型推力：

$$
F_{alloc}=(BU)_F\left(u_{n,final}-u_{n,trim}\right).
$$

水平悬停且高度设定值不变时，模型必须近似满足：

$$
-F_{alloc,z}\simeq mg.
$$

否则切入 INDI 后，模型偏差会立即进入增量控制律。日志中具有清晰切换瞬态的事件如下：

| 日志与事件 | 切入前模型推力/重量 | 观测结果 |
|---|---:|---|
| `log_72` | 0.905 | 推力被削减，约 1 s 掉高 9 cm |
| `log_74` | 0.858 | motor command 从约 0.439 降至 0.145，约 1 s 掉高 15 cm |
| `log_79` 第一次 | 0.977 | 只有较小掉高 |
| `log_79` 第二次 | 1.038 | 小幅上升 |
| `log_81` | 1.085 | motor command 一度升至约 0.615，约 1 s 上升 9 cm |

方向具有一致性：模型推力小于重量时切入后掉高，模型推力大于重量时切入后上升。外环 D 会改变瞬态阻尼，但不能解释这种随模型推力误差改变方向的稳态偏置。

### 8.2 质量、最大推力与 hover thrust 的一致性

22002 只有一个主电机。Allocator 求解所使用的 thrust row 会归一化，因此普通 PID 中 `CA_ROTOR0_CT` 的统一行尺度基本会被 normalization 消去，且积分器可以继续寻找实际悬停油门。Acceleration INDI 的物理反馈则直接使用未归一化的 force matrix：

$$
F_{feedback,z}=-CA\_ROTOR0\_CT\,u.
$$

因此必须满足：

$$
MPC\_THR\_HOVER
\simeq\frac{MPC\_MASS\,g}{CA\_ROTOR0\_CT},
$$

或者：

$$
CA\_ROTOR0\_CT
\simeq\frac{MPC\_MASS\,g}{u_{hover}}.
$$

$u_{hover}$ 应取日志中的 `actuator_motors.control[0]`，不是最终 PWM。定义一致性系数：

$$
K=\frac{MPC\_THR\_HOVER\;CA\_ROTOR0\_CT}
        {MPC\_MASS\,g}.
$$

$K<1$ 表示模型推力不足，存在切入后减推力和掉高倾向；$K>1$ 表示模型推力过强，存在加推力和上升倾向。各参数失配的直接影响为：

| 失配 | Acceleration INDI 中的结果 |
|---|---|
| `CA_ROTOR0_CT` 太小 | 低估当前推力，倾向掉高 |
| `CA_ROTOR0_CT` 太大 | 高估当前推力，倾向上升 |
| `MPC_MASS` 太大 | $F/m$ 偏小，效果类似 CT 太小 |
| `MPC_MASS` 太小 | $F/m$ 偏大，效果类似 CT 太大 |
| `MPC_THR_HOVER` 太小 | acceleration 到 normalized thrust 的输出偏小 |
| `MPC_THR_HOVER` 太大 | normalized thrust 输出偏大 |
| `THR_MDL_FAC` 不匹配 | command 与真实推力的误差随油门变化 |
| 电压变化未建模 | 同一个固定 CT 在不同电压下不再成立 |

本批日志的 `MPC_MASS=1.56 kg`，所以 $mg\simeq15.30 N$。`log_79` 在约 14.38 V 时稳态悬停 command 约为 0.490，对应：

$$
F_{max}\simeq\frac{15.30}{0.490}=31.2 N.
$$

因此 `MPC_MASS=1.56`、`MPC_THR_HOVER=0.49`、`CA_ROTOR0_CT=31.2` 在这一工作点一致。`log_81` 电压降至约 13.90 V 时，悬停 command 上升到约 0.527，对应有效最大推力：

$$
F_{max,eff}\simeq\frac{15.30}{0.527}=29.0 N.
$$

此时继续用固定 `CA_ROTOR0_CT=31.2`，会把真实推力高估约 7%～8%，与切入后上升的方向一致。

### 8.3 推力反馈与加速度反馈的时间对齐

这里的“不匹配”首先是 actuator model 与真实对象之间的动态不匹配。当前 raw force 由 actuator command 直接代入静态矩阵得到：

$$
F_{0,raw}(t)=(BU)_F u_n(t),
$$

但真实推进系统一般满足：

$$
F_{real}(s)=G_{act}(s)F_{0,raw}(s),
$$

其中 $G_{act}(s)$ 同时包含电调、电机、桨的建立过程和可能的纯延迟。常用近似为一阶加纯延迟：

$$
G_{act}(s)\simeq\frac{e^{-s\tau_d}}{T_ms+1},
$$

必要时再使用二阶动态。Low-pass 只能近似 $1/(T_ms+1)$ 这类逐渐建立的动态；纯延迟 $e^{-s\tau_d}$ 在延迟时间内输出完全不变。两者的频率响应不同，不能在较宽频带内互相等价。

日志量化使用以下信号：

1. 将 `allocation_value.raw_allocated_force` 和 `allocated_force` 由 body FRD 转到 NED，并除以 1.56 kg；
2. 将 `vehicle_acceleration` 由 body FRD 转到 NED并补偿重力，作为实际机体加速度的高频代理；
3. 对飞行段去除慢趋势后计算正时延相关，正值表示模型推力领先实际加速度；
4. 仅统计 22002 日志，排除同编号的 1003 日志。

结果为：

| 链路 | 71～82 日志结果 | 解释 |
|---|---|---|
| raw model force → 实际加速度 | 可靠片段主要为 30～60 ms，中位数约 40 ms | 电调、电机、桨、机体和传感器链路使真实加速度晚于 actuator model |
| raw force → filtered force | 相关峰约 10～80 ms | 各日志的 `CA_FORCE_CUTOFF` 在 3、5、8、10、20 Hz 间变化，低 cutoff 产生更大延迟 |
| filtered force → 实际加速度 | 可靠片段约为 -40～+30 ms，中位数接近 0 ms | 正值表示 force 仍领先实际加速度，负值表示 force LP 已经比实际加速度更晚 |

`vehicle_acceleration` 在这些日志中按 50 ms 间隔记录，`vehicle_local_position` 记录频率更低，因此上述结果只能解释为几十毫秒量级，单个日志的 10 ms 数字不具有 10 ms 的测量精度。

这里得到的 30～60 ms 是 raw allocated force 到实际加速度的**等效动态滞后**，不是已经分离出来的纯 dead time。现有日志采样率和自然飞行激励不足以分别辨识 $T_m$ 与 $\tau_d$；需要高频记录并施加安全的小幅阶跃、PRBS 或扫频推力激励，才能拟合一阶/二阶加纯延迟模型。

位置控制器真正使用的 $a_0$ 不是 `vehicle_acceleration`，而是 local-position velocity 经过 `MPC_VEL_LP`、差分和 `MPC_VELD_LP` 得到的量。日志 79～81 使用旧代码的一阶 40 Hz 与一阶 5 Hz，其低频滤波群延迟合计约 36 ms；日志 82 使用一阶 25 Hz 与一阶 5 Hz，约 38 ms。更早日志还试过关闭 velocity LP 或使用更高的 derivative cutoff，不能合并成一个滤波配置。

本次修改后两级均为 `LowPassFilter2p`。若继续采用 40 Hz 与 5 Hz，两个低通的低频群延迟合计约：

$$
\tau_{a_0}\simeq
\frac{\sqrt{2}}{2\pi\,40}
+\frac{\sqrt{2}}{2\pi\,5}
\simeq 51\ \mathrm{ms}.
$$

离散 backward difference 还产生约半个采样周期的相位延迟；position-control 约 100 Hz 时约为 5 ms。因此完整的已知滤波/差分延迟在 40 Hz + 5 Hz 下约为 56 ms；采用 25 Hz + 5 Hz 时约为 59 ms。这里还没有计入 EKF velocity estimator 的频率相关相位。

两条当前反馈链可以展开为：

| $F_0/m$ 链路 | 典型延迟或作用 |
|---|---|
| torque-setpoint 触发 allocator、矩阵回算并发布 raw force | 日志中 `timestamp-timestamp_sample` 中位数约 0.3～0.4 ms |
| `CA_FORCE_CUTOFF` 二阶 LP | 3/5/8 Hz 的低频群延迟分别约 75/45/28 ms |
| position controller 读取 latest `AllocationValue` | 0～一个 position-control 周期的 sample-and-hold，通常小于 10 ms |
| body-to-NED、mass 与 HTE scale | 计算延迟很小；HTE scale 对快速 $F_0$ 变化近似为慢变常数 |
| 未被静态矩阵描述的真实推进系统 | raw force 到实际加速度的等效滞后约 30～60 ms |

| $a_0$ 链路 | 典型延迟或作用 |
|---|---|
| 真实推力产生平动加速度 | $F=ma$ 本身不增加积分环节；主要动态在推进系统 |
| EKF velocity estimate 与 local-position publication | 日志中 publication 计算时间约 1.0～1.8 ms，但 estimator 相位延迟不能由该字段直接得到 |
| `MPC_VEL_LP` 二阶 LP | 40/25 Hz 时低频群延迟约 5.6/9.0 ms |
| backward velocity difference | 100 Hz 时约 5 ms |
| `MPC_VELD_LP` 二阶 LP | 5 Hz 时低频群延迟约 45 ms |

所以消息计算和 uORB 时间只占几毫秒；主要不匹配来自真实推进对象与两条不同的滤波链。额外 force 纯延迟应由三部分共同决定：

$$
\tau_{add}\simeq\tau_{plant}+\tau_{a_0}-\tau_{F0},
$$

其中 $\tau_{plant}$ 是 raw model force 到实际加速度的推进系统延迟，$\tau_{a_0}$ 是实际运动到控制器 filtered acceleration 的延迟，$\tau_{F0}$ 是 `CA_FORCE_CUTOFF` 已经提供的延迟。不能在不知道 cutoff 的情况下固定填写 50 ms。

还可以直接比较最终做差两项的前级信号。日志没有高频记录 `PositionControlStates::acceleration`，所以用 `vehicle_local_position.az` 作为 velocity-derivative acceleration 的代理，并比较：

$$
\frac{F_{0,filtered}}m+g e_z
\quad\text{与}\quad
a_{EKF,z}.
$$

在相关性足够的 22002 片段中，filtered $F_0$ 相对 `local_position.az` 的相关峰约为 -30～+20 ms，中位数约 -10 ms；负值表示 filtered force 比这个 acceleration proxy 更晚。将本次二阶 velocity/derivative filter 和离散差分的 56～59 ms 延迟叠加后，估计最终进入 INDI 做差的 $F_0/m$ 仍领先 $a_0$：

| 日志 | `CA_FORCE_CUTOFF` | filtered $F_0$ → `local_position.az` | 按新二阶 $a_0$ 链估计的 $F_0$ 领先量 |
|---|---:|---:|---:|
| `log_79` | 5 Hz | -10 ms | 约 46 ms |
| `log_80` | 3 Hz | -30 ms | 约 26 ms |
| `log_81` | 3 Hz | -20 ms | 约 36 ms |
| `log_82` | 3 Hz | +10 ms | 约 69 ms |

因此针对最终 INDI 差值，现有证据给出的范围约为 25～70 ms，中心值约 40～45 ms。这个结果由低频日志代理和新滤波器相位推算得到；新二阶链尚未产生对应实飞日志，不能把表中数值当作最终标定结果。

按日志得到的 $\tau_{plant}\simeq40$～$60 ms$，并使用本次二阶 $a_0$ 滤波：

| `CA_FORCE_CUTOFF` | 二阶 LP 低频群延迟近似 | 建议首轮额外纯延迟 |
|---:|---:|---:|
| 3 Hz | 75 ms | 20～40 ms，先试 30 ms |
| 5 Hz | 45 ms | 40～70 ms，先试 50 ms |
| 8 Hz（代码默认值） | 28 ms | 60～80 ms，先试 70 ms |

这些是用于产生下一批高频日志的起点，不是最终参数。降低 force cutoff 也能增加相位滞后，但会同时衰减动态幅值；参数化纯延迟只改变时间关系，因而更适合单独完成同步。

需要特别区分参数方向：**增大** low-pass cutoff 会减小滤波、减小相位滞后并提高带宽；要让模型更“钝”，应当**降低** cutoff 或增大时间常数。但不能为了追求时间对齐而一味降低 cutoff，因为这样会把 INDI 需要补偿的快速 force 变化也衰减掉。

推荐把问题拆成两层：

1. 用 `CA_FORCE_CUTOFF` 或独立 actuator model 近似推进系统的幅值建立过程 $G_{dyn}(s)$；
2. 用带时间戳的环形缓冲区补偿辨识后仍存在的纯延迟和跨链路时间偏差 $e^{-s\tau_d}$；
3. 在 actuator model 之后，对 $F_0/m$ 与测得的 $a_0$ 使用相同阶数、相同 cutoff 的 common INDI low-pass，使两条反馈链具有相同附加相位。

理想结构可以写成：

$$
\hat F_0(s)=e^{-s\hat\tau_d}\hat G_{dyn}(s)(BU)_F u_n(s),
$$

$$
F_{0,INDI}=L(s)\hat F_0,
\qquad
a_{0,INDI}=L(s)a_{meas},
$$

其中 $L(s)$ 是两条链共用的噪声滤波器。当前实现中 force 与 acceleration 使用不同位置、不同阶数的滤波链，因此 `CA_FORCE_CUTOFF` 同时承担了 actuator model 和噪声滤波两个职责，调参会互相牵制。

Rate INDI 也遵循相同原则，但 gyro/angular-acceleration 链运行频率高，角加速度直接由高频 gyro 差分得到，通常没有 translational velocity/EKF 链的额外延迟。Acceleration INDI 的 $a_0$ 来自 local-position velocity 差分，又经过 velocity LP 和 derivative LP，控制频率也更低，所以仅令 `CA_FORCE_CUTOFF` 与 `MPC_VELD_LP` 数值相同并不能保证相位一致。

本次已经增加带时间戳的 force history，而不是阻塞控制循环：

$$
F_{0,aligned}(t)=F_{0,filtered}(t-\tau_F),
$$

参数 `MPC_INDI_F_DLY`（单位 s，范围 0～0.1 s）控制 $\tau_F$。通用默认值为 0；22002 实机 airframe 在 `CA_FORCE_CUTOFF=5 Hz`、22002 Gazebo Classic airframe 在 `CA_FORCE_CUTOFF=6 Hz` 时均使用经本节试验验证的 0.05 s 初值。实现满足：

- 用环形缓冲区选择 `now - delay` 附近的样本，不使用 `sleep` 或降低控制频率；
- body force 必须与同一历史时刻的 attitude 配对后再转到 NED，或者直接缓存当时已转换的 $F_0^n$；
- 对相邻历史样本插值，保留 `AllocationValue` freshness 和 finite 检查；
- 通过 `acceleration_indi_status` 单独记录控制器内部的 filtered $a_0$、未延迟/已延迟 $F_0/m$、所选 force 时间戳、HTE 和 scale，供下一批日志直接辨识最终做差两项；
- 固定纯延迟只修正相位，不能修正 CT、质量或电压造成的幅值错误。

延迟放在 `MulticopterPositionControl` 消费 `AllocationValue.allocated_force` 的链路上：新 allocation 样本到达后，先用当时可用的 attitude 将 body force 转成 NED 并写入历史缓冲；位置控制更新时，以 `vehicle_local_position.timestamp_sample-MPC_INDI_F_DLY` 为目标时间在相邻样本间插值；随后才应用 HTE force scale、除以质量并交给 `PositionControl`。历史不足、样本超过 100 ms 或数据非 finite 时，不使用 INDI feedback。这样延迟的是 $F_0$ 反馈，而不是控制器执行或 actuator command。

0.05 s 对 SITL 已有直接验证，但对实机仍只是根据旧日志 25～70 ms 范围选取的首轮中心值。新 topic 有了直接的内部 $F_0/m$ 和 $a_0$ 后，应通过相关/频响重新调 `MPC_INDI_F_DLY`，目标是两项在 acceleration INDI 有效频带内相位一致，而不是机械地保留 0.05 s。

22002 Gazebo Classic 使用的电机插件对 rotor speed 实现一阶环节

$$
G_\omega(s)=\frac{1}{\tau_m s+1},
$$

默认 `timeConstantUp=timeConstantDown=0.03 s`，实际推力再按 $F=k\omega|\omega|$ 计算。因此 SDF 参数不是纯运输延迟；在悬停点小扰动线性化后，推力大致继承相同的一阶极点。

本次用相同的 3 m 起飞、悬停、降落流程做了对照。ULog 中 `acceleration_indi_status` 以 16 ms 周期记录，分析时对 $F_0/m$ 与 $a_0$ 做 0.15～4 Hz 零相位带通，再搜索互相关峰：

| 电机时间常数 | `MPC_INDI_F_DLY` | 未延迟 $F_0$ 到 $a_0$ 的最佳时移 | 控制器使用的 $F_0$ 到 $a_0$ 的剩余时移 | 零时移相关系数 |
|---:|---:|---:|---:|---:|
| 0.03 s | 0 | 48 ms | 48 ms | 0.815（PID 空中片段） |
| 0.03 s | 0.05 s | 48 ms | 16 ms（一个日志采样周期） | 0.984（完整空中片段） |
| 0.10 s（临时对照） | 0.05 s | 96 ms | 48 ms | 0.925（完整空中片段） |

把电机时间常数临时增加 70 ms 后，辨识出的等效滞后约增加 48 ms；固定 50 ms delay 只能消除其中约 50 ms。试验结束后 SDF 已恢复 0.03 s。结果同时说明两点：环形缓冲实现确实把 force feedback 后移了指定时间；一阶对象的相位随频率变化，不能用一个纯延迟在整个频带完全代替。工程上应先让 `CA_FORCE_CUTOFF` 表示主要执行器建立动态，再用 `MPC_INDI_F_DLY` 消除目标 INDI 频带内剩余的近似固定时移。

闭环效果也做了对照。`MPC_INDI_F_DLY=0` 的 INDI 近悬停连续片段只有 22.4 s，输出随后需要退出 INDI；`0.05 s` 的一直 INDI 飞行得到 78.5 s 连续近悬停片段并正常降落。取各片段进入近悬停后的相同时间窗：

| delay | 时间窗 | 高度峰峰值 | 垂速 RMS | thrust 标准差 | thrust > 0.9 占比 |
|---:|---:|---:|---:|---:|---:|
| 0 | 1～6 s | 0.112 m | 0.182 m/s | 0.216 | 8.0% |
| 0 | 16～21 s | 0.106 m | 0.281 m/s | 0.335 | 21.6% |
| 0.05 s | 1～6 s | 0.094 m | 0.020 m/s | 0.002 | 0% |
| 0.05 s | 16～21 s | 0.023 m | 0.009 m/s | 0.002 | 0% |

无延迟时高度数值尚未立即发散，但 thrust 已在 0～1 间强烈振荡并频繁饱和，垂速 RMS 随时间增大；50 ms 延迟后 thrust 保持在悬停值约 0.51 附近。相位分析同样显示，控制器实际使用的 force 与 $a_0$ 的零时移相关系数由约 0.49 提高到 0.984，最佳剩余时移由 48 ms 降到一个 ULog 采样周期约 16 ms。这里比较的是两次相同 22002 模型的近悬停窗口，但起飞前的控制器启用时刻不同；它足以证明延迟对该模型闭环稳定性有决定性影响，实机参数仍须用实机日志复辨识。

### 8.4 本次运行时 force-feedback scale 修正

本次实现保留 control allocation physical matrix 的标称 `CA_ROTOR0_CT`，不在飞行中改参数或重建矩阵。Hover Thrust Estimator 给出当前悬停 normalized thrust $\hat u_h$ 后，计算有效最大推力：

$$
C_{T,eff}=\frac{mg}{\hat u_h}.
$$

相对于标称矩阵系数 $C_{T,nom}=CA\_ROTOR0\_CT$，只对 acceleration INDI 消费的 force feedback 乘：

$$
s_F=\frac{C_{T,eff}}{C_{T,nom}}
=\frac{mg}{\hat u_h\,CA\_ROTOR0\_CT},
$$

$$
F_{0,corrected}=s_F F_{0,allocation}.
$$

在水平悬停点，若 allocator model 给出：

$$
F_{0,allocation,z}=-CA\_ROTOR0\_CT\,\hat u_h,
$$

则修正后必有：

$$
F_{0,corrected,z}
=-\frac{mg}{\hat u_h\,CA\_ROTOR0\_CT}
 CA\_ROTOR0\_CT\,\hat u_h
=-mg.
$$

这就是修正的数学目的：利用当前悬停油门重新标定 INDI 所看到的 force 尺度，同时保持 allocator 的归一化求解、执行器输出和标称几何矩阵不变。

具体实现位于 `MulticopterPositionControl`：

1. 仅对 physical force 路由明确的 `CA_AIRFRAME=16/17` 启用运行时 scale；
2. 启动和地面阶段从 `MPC_THR_HOVER` 初始化 $\hat u_h$；
3. INDI 实际控制时 valid HTE 首次出现，以 `MPC_INDI_TR_T` 指定的时长从当前应用值 smoothstep 过渡到最新估计；完成首次接管后直接跟踪 valid HTE；普通 PID 不增加这层延迟，而是立即交给原生 `updateHoverThrust()` 无扰应用；
4. 每个过渡周期都用同一个实际应用值 $\hat u_{h,applied}$ 同时更新 `_hover_thrust` 和 $s_F$，避免两个尺度彼此不一致；
5. 将 $s_F$ 限制在 `[0.8, 1.2]`，输入无效时回到 1；
6. INDI 有效时调用 `setHoverThrust()`，不执行 `updateHoverThrust()` 的垂向积分器补偿；普通 PID 模式保留原 `updateHoverThrust()` 行为。

曾考虑在 HTE 输出后额外增加 `0.05 normalized thrust/s` 的硬斜率限制，但 71～82 的 22002 日志没有记录 `hover_thrust_estimate` topic，因而没有数据证明 HTE valid 输出存在需要该限制的抖动或跳变；PX4 原 HTE 也没有这层硬限制。为避免未经验证的保护导致 HTE/force-scale 修正长时间跟不上真实变化，本次最终代码没有加入该 limiter。

逐个检查这 12 个 ULog 后确认，缺少该 topic 不是 logger 列表遗漏：PX4 默认 logger 原本就请求 `hover_thrust_estimate`，但 HTE 只有在 `vehicle_local_position.dist_bottom > 1 m` 后才将 `_in_air` 置位并开始发布。日志 71～82 的最大 `dist_bottom` 仅为 0.02～0.91 m，因此全部为 0 个 HTE 样本；这些飞行中运行时 scale 实际一直使用 `MPC_THR_HOVER`。下一次实飞需要至少越过 1 m 一次，之后即使降到 1 m 以下，HTE 仍会继续工作，直到 land detector 再次报告 landed。

一次专门的 22002 SITL 获取试验将 `MPC_THR_HOVER` 故意从真实值约 0.511 改为 0.35，执行起飞到 3 m、稳定悬停和降落。HTE 首次发布即为 0.51795（但 `valid=false`），1.656 s 后以 0.52210 变为 valid；valid 边沿相邻 HTE 样本只变化 $1.1\times10^{-5}$，整段最大相邻变化约 0.00169，说明发布后的估计本身平滑。但 controller 实际使用的 `_indi_hover_thrust` 在 valid 边沿从 0.35 一次变为 0.52210，force scale 同时从限幅后的 1.2 变为 0.9781。这个试验证明需要关注的是**首次 valid 接管跳变**，而不是 HTE 稳态噪声；若增加保护，应只处理 valid 上升沿并参数化过渡时间，不能据此给所有 HTE 更新固定套用 `0.05/s` 限速。

HTE 自身仍包含估计器的过程噪声、测量噪声、状态/创新检查和 `valid` 判定；`HTE_HT_NOISE`（默认 `0.0036 normalized thrust/s`）是过程噪声参数，会影响收敛速度和协方差，但不是最大变化斜率。现在只有 valid 的首次接管经过有限时长过渡，之后 HTE 直接进入 `_hover_thrust` 与 $s_F$，外层只保留物理/安全边界 `[0.8,1.2]`。HTE 目标由原有 `hover_thrust_estimate.hover_thrust` 记录；`acceleration_indi_status.hover_thrust`、过渡进度和 `force_feedback_scale` 只补充控制器内部实际应用值。下一批日志若确实显示 HTE 高频噪声调制控制，再根据数据决定是否增加参数化低通或 slew，而不是预先固定 0.05/s。

### 8.5 PID/acceleration INDI 与 HTE 的无扰过渡

切换控制器时，PID 与 INDI 在同一状态下计算出的 raw normalized thrust 通常不同。直接切换会把这个差值作为执行器阶跃；但在两套控制律之间长期混合又会改变 INDI 本身。当前实现采用有限时长的 output tracking：

$$
u(t)=u_{new,raw}(t)+[1-S(r)]\Delta u_0,
\qquad
S(r)=3r^2-2r^3,
\qquad
r=\operatorname{sat}\!\left(\frac{t-t_s}{T_{tr}}\right),
$$

其中 $\Delta u_0=u_{actual}(t_s^-)-u_{new,raw}(t_s)$，$T_{tr}=MPC\_INDI\_TR\_T$。所以切换首周期严格从上一周期经过公共推力/倾角限制后实际发送的 `thrust_setpoint` 开始，随后 offset 以两端斜率为零的 smoothstep 衰减；过渡结束后输出完全等于新控制器 raw output。PID→INDI、INDI→PID、feedback 失效 fallback，以及过渡中反向切换均使用同一机制。

INDI 有效期间，速度环 I/D 不进入 INDI 的 $a_c$，但切入 INDI **不清零**速度积分器。原 PID 每周期继续运行并保持既有积分状态，行为与 rate INDI 的 hot-standby 结构一致；无扰性由 output tracking 直接约束执行器实际连续性。公共 thrust/tilt saturation 在合成后统一执行，状态 topic 同时记录 `raw_thrust_setpoint`、最终 `thrust_setpoint`、progress 和 active flag。

HTE 首次 valid 且 INDI 实际控制时使用相同的 `MPC_INDI_TR_T`，但过渡对象是 hover-thrust mapping 和匹配的 force scale，而不是控制律输出。二者始终由同一 applied hover thrust 计算，因此不会出现 `_hover_thrust` 已变化而 $F_0$ scale 尚未变化的尺度分裂。若 PID 正在控制或 INDI 因反馈无效已回退 PID，则不额外执行一秒 HTE 插值，直接通过 `updateHoverThrust()` 修改垂向积分器来维持 normalized thrust；以后再切入 INDI 时，从 PID 已经应用的同一 hover thrust 和 force scale 开始。INDI 阶段只调用 `setHoverThrust()`，避免把补偿注入一个不参与 $a_c$ 的积分器。

SITL 验证结果：

- 3 m 悬停时 PID→INDI 的首个已记录样本，最终 z-thrust 变化仅 `0.000182`，随后 100 ms 均值变化 `0.000315`；切换后 1.2 s 高度峰峰值 `0.008 m`、最大垂向速度 `0.039 m/s`；
- `MPC_INDI_F_DLY=0` 时切换本身无扰，但约 22 s 后因 $F_0/a_0$ 相位未对齐逐渐振荡，证明无扰切换不能代替闭环相位匹配；
- `MPC_INDI_F_DLY=0.05 s` 时，从地面一直启用 INDI 完成 3 m 起飞、悬停和降落，近悬停高度 5%～95% 为 `2.969～3.018 m`；
- 同一飞行中 HTE 首次 valid 的目标约为 `0.52250`，实际应用 hover thrust 从 `0.51067` 在 1 s 内平滑移向目标；首次接管附近最大垂向速度 `0.161 m/s`，没有出现 normalized thrust 阶跃。

为了不把“模型刚好匹配”误认为无扰机制有效，另做了两组强失配 A/B。控制器切换试验在 0.8 m（低于 HTE 启用高度）悬停，并故意设置 `MPC_THR_HOVER=0.40`，使 PID 与 INDI raw z-thrust 相差约 0.03：

| `MPC_INDI_TR_T` | raw z-thrust 差 | 最终 z-thrust 首样本变化 | 100 ms 均值变化 | 切换后 1.2 s 高度峰峰值 |
|---:|---:|---:|---:|---:|
| 0 s | +0.02947 | +0.02947 | +0.02889 | 0.132 m |
| 1 s | +0.03126 | -0.00016 | -0.00028 | 0.068 m |

两次 raw mismatch 相当，但 smoothstep 消除了约 99.5% 的首样本输出阶跃，高度扰动约减少 48%。因此它不是只让 status progress 看起来平滑，而是实际发送的 normalized thrust 连续。

HTE A/B 从地面一直启用 INDI，使用 `MPC_THR_HOVER=0.35` 起飞到 3 m；首次 valid 时两次状态接近，均在高度约 2.02 m、上升速度约 0.264 m/s：

| `MPC_INDI_TR_T` | applied hover 首样本变化 | force scale 首样本变化 | z-thrust 首样本变化 | 100 ms 平均 thrust 变化 | 后续 1.2 s 最大向上加速度 |
|---:|---:|---:|---:|---:|---:|
| 0 s | +0.16766 | -0.21354 | -0.13604 | -0.12371 | 2.07 m/s² |
| 1 s | +0.00003 | 约 0 | +0.00255 | -0.00198 | 1.30 m/s² |

首次 valid 过渡把 thrust 首样本冲击减少约 98.1%，100 ms 平均扰动减少约 98.4%。这里的初值偏差是故意放大的验证工况；正常 `MPC_THR_HOVER≈0.511` 时收益更小，但机制可防止实机初值、电池或模型失配在 HTE 接管瞬间直接变成推力阶跃。

上述数值来自 22002 SITL ULog，仅用于验证实现和辨识流程；实机必须从新日志重新确认 delay、scale 与过渡时间。

## 9. 反馈发布、uORB freshness 与日志

Allocator 在以下条件组合下发布 `AllocationValue`：

- effectiveness source 为 CA0、CA9、CA16 或 CA17；
- CA0/9 配置一个 allocation matrix，CA16/17 配置两个 allocation matrices；
- actuator control publication 处于 active 状态；
- torque setpoint callback 触发完整 allocation 周期。

`uORB::Subscription::copy()` 读取 topic 当前槽位中的最新样本。较低频率的 position controller 会读取调用时刻的最新 `AllocationValue`，中间样本由 uORB latest-value 语义自然合并。

100 ms freshness check 表达 publisher liveness horizon。Publisher 停止后，topic 槽位继续保存最后样本；timestamp age 在 100 ms 后把两个 INDI 控制器切换到 PX4 原路径。`timestamp_sample` 保存反馈与触发 torque sample 的关联，实机日志可以直接统计正常消息年龄。

默认日志配置为：

| Topic | 默认记录策略 | 内容 |
|---|---|---|
| `control_allocator_status` | 2 instances，200 ms | solver、fallback、unallocated control、timing |
| `allocation_value` | 1 instance，20 ms | raw/filtered physical wrench、torque scale |
| `rate_ctrl_status` | 2 instances，200 ms | PID integrator 与 `indi_active` |
| `vehicle_local_position_setpoint` | 100 ms | position-control output 与 `acc_indi_active` |
| `acceleration_indi_status` | 发布速率 | $a_0$、延迟前后 $F_0/m$、实际应用 HTE/scale、切换进度与 `feedback_valid`；最终 thrust/active 分别读取下一行既有字段 |
| `vehicle_torque_setpoint` | 2 instances，20 ms | total torque、PCA higher 与 valid |
| `actuator_motors`、`actuator_servos` | 原有策略 | 最终 actuator output |

High-rate logger profile 以发布速率记录 `allocation_value`、`vehicle_torque_setpoint`、`actuator_motors` 和 `actuator_servos`。

## 10. 适用范围、标定责任与硬件资源

### 10.1 INDI 物理配置契约

INDI 配置包含以下事实：

1. torque rows 使用 N·m，force rows 使用 N；
2. `linearization_point=0`；
3. trim 与 normalized final actuator value 使用同一坐标定义；
4. CA0/9 的 force 与 torque 来自 matrix 0；
5. CA16/17 的 force 来自 matrix 0，torque 来自 matrix 1；
6. `MC_J_X/Y/Z` 与 `MPC_MASS` 采用实机参数；
7. CA0/9 的配置者负责确认 physical $BU$ 标定。

软件支持的 CA type 表达固定反馈路由能力，机型参数表达物理单位标定，`MC_INDI_RATE_EN` 与 `MPC_INDI_ACC_EN` 表达控制器选择。三者共同形成完整启用条件。

`AllocationValue` 来源为最终 actuator setpoint 经过 physical matrix 的模型回算。其作用是表达 allocator 实际交付的模型 wrench；机体参数、effectiveness 标定和 actuator dynamics 决定模型与实机之间的精度。

INV、DP_LPCA 与 DPscaled_LPCA 的适用范围覆盖更多 effectiveness source，因为这些算法只依赖 normalized solver matrix。PCA 进一步依赖 rate INDI 生成的 explicit priority split 与 R/P/Y active axes。

### 10.2 Work queue 与 FMUv5 配置

本分支在 `platforms/common/px4_work_queue/Kconfig` 调整全局默认栈：

| Work queue | main 默认值 | 当前默认值 |
|---|---:|---:|
| `rate_ctrl` | 3150 B | 5750 B |
| SPI | 2392 B | 2092 B |
| I2C | 2336 B | 2036 B |
| INS | 6000 B | 4500 B |
| `lp_default` | 3500 B | 3000 B |

各 board 可以通过自身 Kconfig 覆盖这些默认值。FMUv5 实机测试使用 `top once` 观察 `wq:rate_ctrl`、SPI、I2C、INS、`lp_default` 的 `USED/STACK`，并覆盖启动、解锁、传感器初始化、日志、EKF、最大分配矩阵和持续饱和。

FMUv5 当前 board target 为 flash 预算裁剪 DShot、UAVCAN、camera capture/trigger/feedback、ADIS16448、attitude_estimator_q、FW/MC autotune、gimbal 和 SIH。这个 target 对应本项目硬件功能集合。单实例 EKF 可以进一步减少运行资源占用，work queue high-water 数据负责验证各 queue 的栈容量。

### 10.3 验证入口

本项目使用以下验证入口：

- Host allocation test：`ControlAllocationLPCATest`；
- SITL build 与机型启动：`make px4_sitl gz_ductedfan4`；
- FMUv5 build：`make px4_fmu-v5_default`；
- Runtime diagnostics：`control_allocator status`；
- FMUv5 stack high-water：`top once`；
- Flight log：raw/filtered `AllocationValue`、INDI active flags、torque priority split、actuator output。

## 11. 代码位置

- `src/lib/control_allocation/control_allocation/ControlAllocation.*`：控制设定值、priority component 与诊断基类；
- `src/lib/control_allocation/control_allocation/ControlAllocationLPCA.*`：INV、DP_LPCA、DPscaled_LPCA、PCA adapter；
- `src/lib/control_allocation/control_allocation/pca/ControlAllocation.h`：LPCA/PCA solver；
- `src/lib/control_allocation/control_allocation/ControlAllocationLPCATest.cpp`：host 数值与 fallback test；
- `src/modules/control_allocator/ControlAllocator.*`：方法选择、matrix 配置、timing、物理反馈与滤波；
- `src/modules/control_allocator/VehicleActuatorEffectiveness/*DuctedFan*`：CA16、CA17 effectiveness；
- `msg/ControlAllocatorStatus.msg`：低频 solver 与 allocation diagnostics；
- `msg/AllocationValue.msg`：高速 physical allocation feedback；
- `msg/VehicleTorqueSetpoint.msg`：PCA higher component 与 valid；
- `src/modules/mc_rate_control/MulticopterRateControl.*`：PID hot standby、rate INDI selection、PCA split；
- `src/modules/mc_rate_control/IndiControl/*`：rate INDI control law；
- `src/modules/mc_pos_control/MulticopterPositionControl.*`：force subscription、freshness、body-to-NED 与 inverse mass；
- `src/modules/mc_pos_control/PositionControl/*`：acceleration INDI insertion 与原 thrust mapping；
- `src/modules/vtol_att_control/tailsitter.*`：PX4 tailsitter routing 与 CA17 torque extension；
- `src/modules/logger/logged_topics.cpp`：default 与 high-rate logging；
- `platforms/common/px4_work_queue/Kconfig`：work queue default stack；
- `boards/px4/fmu-v5/default.px4board`：FMUv5 target feature set。
