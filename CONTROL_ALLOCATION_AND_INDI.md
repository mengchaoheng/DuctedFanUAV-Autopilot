# 控制分配与 INDI

本文按代码来源组织内容：

- 第 1、2 节解析 PX4 内置的 $BU$ effectiveness、trim、归一化控制和 allocation matrix 机制；
- 第 3、4 节说明本分支新增的 INV、LPCA、PCA、诊断、运行时间和物理反馈；
- 第 5 节先解析 PX4 内置 VTOL 路由，再说明 CA16、CA17 的项目扩展；
- 第 6、7、8、9 节说明 rate INDI、acceleration INDI、消息日志和硬件配置。

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

`PositionControl` 启动时从 `MPC_THR_HOVER` 初始化 $T_h$，有效 `hover_thrust_estimate` 在线更新 $T_h$，ground phase 使用原 takeoff logic 恢复参数值。当前生效的 hover thrust 完成 acceleration 到 normalized thrust 的尺度转换。

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

## 8. 反馈发布、uORB freshness 与日志

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
| `vehicle_torque_setpoint` | 2 instances，20 ms | total torque、PCA higher 与 valid |
| `actuator_motors`、`actuator_servos` | 原有策略 | 最终 actuator output |

High-rate logger profile 以发布速率记录 `allocation_value`、`vehicle_torque_setpoint`、`actuator_motors` 和 `actuator_servos`。

## 9. 适用范围、标定责任与硬件资源

### 9.1 INDI 物理配置契约

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

### 9.2 Work queue 与 FMUv5 配置

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

### 9.3 验证入口

本项目使用以下验证入口：

- Host allocation test：`ControlAllocationLPCATest`；
- SITL build 与机型启动：`make px4_sitl gz_ductedfan4`；
- FMUv5 build：`make px4_fmu-v5_default`；
- Runtime diagnostics：`control_allocator status`；
- FMUv5 stack high-water：`top once`；
- Flight log：raw/filtered `AllocationValue`、INDI active flags、torque priority split、actuator output。

## 10. 代码位置

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
