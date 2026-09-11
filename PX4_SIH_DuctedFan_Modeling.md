# PX4 SIH 涵道飞行器建模说明

> 适用分支：`df-mpc`  
> 机型：`ductedfan4`、`SHC09`、`SHW09_vtol`  
> 实际采用两类 SIH plant：
>
> 1. `DuctedFan`：`ductedfan4` 与 `SHC09`
> 2. `DuctedFanTailsitter`：`SHW09_vtol`

---

## 1. 建模目标

本项目中的 SIH 主要用于验证完整 PX4 飞控逻辑：

```text
EKF2
  ↓
位置 / 姿态 / Rate 控制
  ↓
INDI / PID
  ↓
Control Allocator
  ↓
执行器输出
  ↓
simulator_sih
  ↓
刚体运动 + 模拟传感器
  ↓
EKF2
```

因此 SIH 模型采用轻量化策略：

- 保留刚体 6-DOF 动力学；
- 保留主风扇推力；
- 保留舵面主要控制力矩；
- `SHW09_vtol` 增加简化主翼升阻力；
- 执行器采用一阶动态；
- 传感器、EKF2、VTOL 状态机、Control Allocation、INDI 等继续运行 PX4 原始代码；
- 高保真的涵道侧力、复杂非定常气动、详细风扇反扭矩等继续留给 GZ / Gazebo Classic。

这使 SIH 的重点保持在“完整飞控软件逻辑”，同时保留足够的物理真实性。

---

# 2. 统一坐标系与刚体动力学

PX4 机体系采用 FRD：

```text
+x：前
+y：右
+z：下
```

对于竖直悬停的涵道飞行器，主风扇推力沿机体 `-z` 方向。

主推力向量为：

\(\displaystyle
\mathbf T_B=
\begin{bmatrix}
0\\
0\\
-T
\end{bmatrix}.
\)

统一平动模型可写为：

\(\displaystyle
m\dot{\mathbf v}_N
=
m\mathbf g_N
+
\mathbf R_B^N
\left(
\mathbf T_B+\mathbf F_{a,B}
\right)
+
\mathbf F_{\mathrm{ground},N}.
\)

统一转动模型为：

\(\displaystyle
\mathbf I\dot{\boldsymbol\omega}_B
=
\mathbf M_B
-
\boldsymbol\omega_B
\times
\left(
\mathbf I\boldsymbol\omega_B
\right).
\)

其中：

\(\displaystyle
\mathbf I=
\mathrm{diag}(J_x,J_y,J_z).
\)

姿态由四元数积分：

\(\displaystyle
\dot{\mathbf q}
=
\frac{1}{2}
\mathbf q
\otimes
\begin{bmatrix}
0\\
\boldsymbol\omega_B
\end{bmatrix}.
\)

---

# 3. 执行器动态

当前 SIH 中电机和舵机均采用一阶执行器模型：

\(\displaystyle
\dot u
=
\frac{u_{\mathrm{cmd}}-u}{\tau}.
\)

离散实现近似为：

\(\displaystyle
u_{k+1}
=
u_k
+
\frac{\Delta t}{\tau}
\left(
u_{\mathrm{cmd},k}-u_k
\right).
\)

SIH 主循环频率为 250 Hz：

\(\displaystyle
\Delta t=0.004\ {\rm s}.
\)

当前参数：

| 机型 | 电机/舵机统一时间常数 |
|---|---:|
| `ductedfan4` | \(\displaystyle \tau=0.03\ {\rm s}\) |
| `SHC09` | \(\displaystyle \tau=0.03\ {\rm s}\) |
| `SHW09_vtol` | \(\displaystyle \tau=0.025\ {\rm s}\) |

当前第一版仍然让同一机型的电机与舵机共用一个时间常数。后续可以进一步拆成 \(\tau_m\) 与 \(\tau_s\)。

---

# 4. 模型一：DuctedFan

适用机型：

```text
ductedfan4
SHC09
```

两者使用完全相同的数学结构，仅质量、惯量、最大推力和舵面 effectiveness 参数不同。

SIH 类型：

```text
SIH_VEHICLE_TYPE = 6
```

---

## 4.1 主风扇模型

归一化电机输出：

\(\displaystyle
0\le u_m\le1.
\)

由于当前 airframe 使用：

```text
THR_MDL_FAC = 1
```

SIH 将电机输出解释为归一化转速，并采用二次推力模型：

\(\displaystyle
T=T_{\max}u_m^2.
\)

因此：

\(\displaystyle
\mathbf T_B=
\begin{bmatrix}
0\\
0\\
-T_{\max}u_m^2
\end{bmatrix}.
\)

最大推力直接读取：

```text
CA_ROTOR0_CT
```

SIH 中 `SIH_T_MAX` 仅作为 fallback。

---

## 4.2 舵面控制力矩

舵面归一化输入：

\(\displaystyle
-1\le u_i\le1.
\)

第 \(i\) 个舵面的物理控制力矩向量定义为：

\(\displaystyle
\mathbf b_i=
\begin{bmatrix}
b_{R,i}\\
b_{P,i}\\
b_{Y,i}
\end{bmatrix}
=
\begin{bmatrix}
\texttt{CA\_SV\_CSi\_TRQ\_R}\\
\texttt{CA\_SV\_CSi\_TRQ\_P}\\
\texttt{CA\_SV\_CSi\_TRQ\_Y}
\end{bmatrix}.
\)

单位为 N·m，并表示归一化舵面指令 \(u_i=+1\) 时的物理力矩。

总控制力矩：

\(\displaystyle
\mathbf M_c
=
\sum_{i=1}^{N_s}
\mathbf b_i u_i.
\)

其中：

- `ductedfan4`：\(N_s=4\)
- `SHC09`：\(N_s=6\)

这个模型的特点是：

\(\displaystyle
B_{\mathrm{plant}}=B_{\mathrm{CA}}
\)

在第一版中，SIH plant 与 Control Allocator 使用同一套 `CA_SV_CS*` 物理力矩参数。

---

# 5. ductedfan4

## 5.1 物理参数

\(\displaystyle
m=1.56\ {\rm kg}.
\)

\(\displaystyle
J_x=0.01149\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_y=0.01153\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_z=0.00487\ {\rm kg\,m^2}.
\)

最大推力：

\(\displaystyle
T_{\max}=30\ {\rm N}.
\)

---

## 5.2 主推力

\(\displaystyle
T=30u_m^2.
\)

理论悬停满足：

\(\displaystyle
30u_{m,h}^2=mg.
\)

所以：

\(\displaystyle
u_{m,h}
=
\sqrt{\frac{1.56\times9.80665}{30}}
\approx0.714.
\)

对应推力比例：

\(\displaystyle
u_{m,h}^2\approx0.510.
\)

---

## 5.3 四舵面力矩

定义四个舵面输入：

\(\displaystyle
u_1,u_2,u_3,u_4\in[-1,1].
\)

当前物理力矩矩阵等价于：

\(\displaystyle
M_x
=
0.166227(-u_1+u_3).
\)

\(\displaystyle
M_y
=
0.166227(-u_2+u_4).
\)

\(\displaystyle
M_z
=
0.068681(u_1+u_2+u_3+u_4).
\)

因此：

\(\displaystyle
\mathbf M_c=
\begin{bmatrix}
0.166227(-u_1+u_3)\\
0.166227(-u_2+u_4)\\
0.068681(u_1+u_2+u_3+u_4)
\end{bmatrix}.
\)

第一版 SIH 中不进一步计算涵道尾流速度对舵效的变化。

---

# 6. SHC09

## 6.1 物理参数

\(\displaystyle
m=2.10\ {\rm kg}.
\)

\(\displaystyle
J_x=0.0438\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_y=0.0436\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_z=0.005006\ {\rm kg\,m^2}.
\)

最大推力：

\(\displaystyle
T_{\max}=42.3016\ {\rm N}.
\)

---

## 6.2 主推力

\(\displaystyle
T=42.3016u_m^2.
\)

理论悬停：

\(\displaystyle
u_{m,h}
=
\sqrt{
\frac{2.1\times9.80665}{42.3016}
}
\approx0.698.
\)

所以：

\(\displaystyle
u_{m,h}^2\approx0.487.
\)

这与 airframe 中：

```text
MPC_THR_HOVER = 0.4865
```

基本一致。

---

## 6.3 六舵面力矩

定义：

\(\displaystyle
u_1,\ldots,u_6\in[-1,1].
\)

滚转力矩：

\(\displaystyle
M_x=
-0.343572u_1
-0.171786u_2
+0.171786u_3
+0.343572u_4
+0.171786u_5
-0.171786u_6.
\)

俯仰力矩：

\(\displaystyle
M_y=
0.297542u_2
+0.297542u_3
-0.297542u_5
-0.297542u_6.
\)

偏航力矩：

\(\displaystyle
M_z=
0.084928
(u_1+u_2+u_3+u_4+u_5+u_6).
\)

因此 SHC09 与 ductedfan4 的 SIH 数学结构完全一致：

\(\displaystyle
T=T_{\max}u_m^2,
\qquad
\mathbf M_c=\sum_i\mathbf b_i u_i.
\)

---

# 7. DuctedFan 模型的空气阻尼

通用 SIH 保留线性阻尼接口：

\(\displaystyle
\mathbf F_a=-K_{DV}\mathbf v_a.
\)

\(\displaystyle
\mathbf M_a=-K_{DW}\boldsymbol\omega.
\)

当前：

```text
ductedfan4:
SIH_KDV = 0
SIH_KDW = 0

SHC09:
SIH_KDV = 0
SIH_KDW = 0
```

因此这两个机型的第一版 plant 主要由：

```text
主推力
+ 舵面控制力矩
+ 重力
+ 刚体惯性
+ 地面接触
```

组成。

---

# 8. 模型二：DuctedFanTailsitter

适用机型：

```text
SHW09_vtol
```

SIH 类型：

```text
SIH_VEHICLE_TYPE = 7
```

SHW09_vtol 与前两个机型的主要区别在于：

- 单涵道主风扇；
- 6 个涵道内舵面；
- 2 个固定翼 elevon；
- 尾座式 VTOL；
- 悬停、转换和固定翼阶段的实际舵效会随着流场连续变化；
- 固定翼阶段需要主翼升力和阻力。

因此 SHW09_vtol 使用独立的轻量化尾座式模型。

---

# 9. SHW09_vtol 基本参数

质量：

\(\displaystyle
m=2.05\ {\rm kg}.
\)

惯量：

\(\displaystyle
J_x=0.050636\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_y=0.042954\ {\rm kg\,m^2}.
\)

\(\displaystyle
J_z=0.012668\ {\rm kg\,m^2}.
\)

最大风扇推力：

\(\displaystyle
T_{\max}=32.1984\ {\rm N}.
\)

主风扇：

\(\displaystyle
T=32.1984u_m^2.
\)

理论悬停电机归一化转速：

\(\displaystyle
u_{m,h}
=
\sqrt{
\frac{2.05\times9.80665}{32.1984}
}
\approx0.790.
\)

对应推力比例：

\(\displaystyle
u_{m,h}^2\approx0.624.
\)

与：

```text
MPC_THR_HOVER = 0.62
```

基本一致。

---

# 10. SHW09_vtol 固定翼坐标

尾座式飞机悬停时主风扇沿 body `-z` 推进。

进入平飞后，固定翼前向轴对应：

\(\displaystyle
\mathbf e_{x,\mathrm{FW}}
=
-\mathbf e_{z,B}.
\)

因此 SIH 中固定翼前向空速采用：

\(\displaystyle
V_a
=
\max(0,-v_{B,z}).
\)

这个定义同时用于：

- 固定翼气动力；
- elevon 舵效；
- simulated airspeed。

---

# 11. SHW09_vtol 涵道舵面 CS0–CS5

SHW09_vtol 的前 6 个舵面位于涵道出口流场中。

Gazebo / GZ 的基本关系可以简化为：

\(\displaystyle
F_s
\propto
q_e\delta
\)

其中：

\(\displaystyle
q_e
=
\frac{1}{2}\rho V_e^2.
\)

因此舵面控制力矩满足：

\(\displaystyle
\mathbf M_{\mathrm{duct}}
\propto
V_e^2\delta.
\)

---

## 11.1 悬停参考出口速度

采用已验证模型中的悬停参考：

\(\displaystyle
V_{e,h}=24.1618\ {\rm m/s}.
\)

当前 SIH 使用主风扇归一化转速估算 propwash：

\(\displaystyle
V_{\mathrm{prop}}
=
V_{e,h}
\frac{u_m}{u_{m,h}}.
\)

由于 GZ / Gazebo Classic 中涵道舵面采用自由来流与风扇尾流叠加的处理，SIH 简化为：

\(\displaystyle
V_e
=
V_{\mathrm{prop}}+V_a.
\)

于是涵道舵面的连续 effectiveness scale 为：

\(\displaystyle
k_{\mathrm{duct}}
=
\mathrm{sat}
\left[
\left(
\frac{V_e}{V_{e,h}}
\right)^2,
0,
3
\right].
\)

其中 `sat` 表示限幅。

---

## 11.2 CS0–CS5 参考力矩

定义六个涵道舵输入：

\(\displaystyle
u_1,\ldots,u_6\in[-1,1].
\)

参考滚转力矩：

\(\displaystyle
M_{x,0}
=
-0.704019u_1
-0.352009u_2
+0.352009u_3
+0.704019u_4
+0.352009u_5
-0.352009u_6.
\)

参考俯仰力矩：

\(\displaystyle
M_{y,0}
=
0.609698
(u_2+u_3-u_5-u_6).
\)

参考偏航力矩：

\(\displaystyle
M_{z,0}
=
0.194941
(u_1+u_2+u_3+u_4+u_5+u_6).
\)

实际 SIH 涵道舵控制力矩为：

\(\displaystyle
\mathbf M_{\mathrm{duct}}
=
k_{\mathrm{duct}}
\begin{bmatrix}
M_{x,0}\\
M_{y,0}\\
M_{z,0}
\end{bmatrix}.
\)

因此悬停、转换和固定翼之间的涵道舵效是连续变化的。

---

# 12. SHW09_vtol 固定翼舵面 CS6–CS7

CS6 和 CS7 是左右固定翼 elevon。

其气动力主要由自由来流动态压强决定：

\(\displaystyle
q_\infty
=
\frac{1}{2}\rho V_a^2.
\)

选择巡航参考空速：

\(\displaystyle
V_{a,\mathrm{ref}}
=
16.4911\ {\rm m/s}.
\)

定义：

\(\displaystyle
k_{\mathrm{wing}}
=
\mathrm{sat}
\left[
\left(
\frac{V_a}{V_{a,\mathrm{ref}}}
\right)^2,
0,
3
\right].
\)

当前 CA 参考力矩：

\(\displaystyle
M_{z,6}
=
1.240193u_7.
\)

\(\displaystyle
M_{z,7}
=
-1.240193u_8.
\)

因此 elevon 在 SIH 中产生：

\(\displaystyle
M_{z,\mathrm{elevon}}
=
1.240193
k_{\mathrm{wing}}
(u_7-u_8).
\)

由于尾座式坐标关系，body-\(z\) 力矩在平飞状态主要对应固定翼滚转控制。

悬停时：

\(\displaystyle
V_a\approx0
\)

所以：

\(\displaystyle
k_{\mathrm{wing}}\approx0.
\)

因此 CS6/CS7 会自然失去大部分气动控制能力。

---

# 13. CA17 与 SIH plant 的关系

这是 SHW09_vtol 建模中最重要的分层。

Control Allocator 中仍然保留：

```text
DF_FW_CS_GAIN = 1.8393
```

CA17 根据 VTOL flight phase 修改自己的 effectiveness matrix：

```text
Hover:
CS0..5 工作
CS6..7 关闭

Transition / FW:
CS0..5 × DF_FW_CS_GAIN
CS6..7 激活
```

它属于控制器内部的执行器能力估计：

\(\displaystyle
B_{\mathrm{CA}}
=
B_{\mathrm{CA}}(\text{flight phase}).
\)

SIH plant 则取消相同的硬状态倍乘，改成流速连续变化：

\(\displaystyle
B_{\mathrm{plant}}
=
B_{\mathrm{plant}}
(u_m,V_a).
\)

因此：

\(\displaystyle
B_{\mathrm{CA}}
\approx
B_{\mathrm{plant}},
\)

同时允许合理的模型失配。

这也更适合测试 INDI 对 effectiveness mismatch 的鲁棒性。

---

# 14. 为什么 DF_FW_CS_GAIN 约为 1.84

从已经验证的 SHW09_vtol 模型中：

悬停出口速度：

\(\displaystyle
V_{e,h}=24.1618\ {\rm m/s}.
\)

典型巡航空速：

\(\displaystyle
V_a\approx16.4911\ {\rm m/s}.
\)

典型巡航风扇滑流：

\(\displaystyle
V_{\mathrm{prop}}\approx16.45\ {\rm m/s}.
\)

所以巡航涵道出口总流速近似为：

\(\displaystyle
V_{e,\mathrm{FW}}
\approx
16.4911+16.45
\approx32.94\ {\rm m/s}.
\)

对应舵效比：

\(\displaystyle
\frac{k_{\mathrm{FW}}}{k_{\mathrm{hover}}}
\approx
\left(
\frac{32.94}{24.1618}
\right)^2
\approx1.86.
\)

这与：

\(\displaystyle
\texttt{DF\_FW\_CS\_GAIN}=1.8393
\)

非常接近。

因此 CA17 的状态增益与真实流速平方关系具有一致的物理量级。

---

# 15. SHW09_vtol 主翼模型

SHW09_vtol SIH 使用两片独立主翼，而不是把两片机翼完全 lump 到质心。

单侧机翼面积：

\(\displaystyle
S_w=0.115\ {\rm m^2}.
\)

左右翼压力中心：

\(\displaystyle
\mathbf r_L=
\begin{bmatrix}
0\\
+0.3145\\
0
\end{bmatrix}
{\rm m},
\qquad
\mathbf r_R=
\begin{bmatrix}
0\\
-0.3145\\
0
\end{bmatrix}
{\rm m}.
\)

气体密度：

\(\displaystyle
\rho=1.2041\ {\rm kg/m^3}.
\)

---

## 15.1 翼面局部速度

先把机体速度转换到固定翼气动坐标系：

\(\displaystyle
\mathbf v_{\mathrm{FW}}
=
\mathbf R_{S}^{B\,T}
\mathbf v_B.
\)

\(\displaystyle
\boldsymbol\omega_{\mathrm{FW}}
=
\mathbf R_{S}^{B\,T}
\boldsymbol\omega_B.
\)

左右翼局部来流考虑角速度产生的速度：

\(\displaystyle
\mathbf v_i
=
\mathbf v_{\mathrm{FW}}
+
\boldsymbol\omega_{\mathrm{FW}}
\times
\mathbf r_i.
\)

这一步使左右翼在滚转过程中具有不同局部来流，从而自然形成一定的滚转气动阻尼。

---

## 15.2 二维升阻力平面

对每个机翼，仅取其 lift-drag plane 内速度：

\(\displaystyle
\mathbf v_{LD,i}
=
\begin{bmatrix}
v_{x,i}\\
0\\
v_{z,i}
\end{bmatrix}.
\)

速度大小：

\(\displaystyle
V_i
=
\left\|
\mathbf v_{LD,i}
\right\|.
\)

迎角：

\(\displaystyle
\alpha_i
=
\alpha_0
+
\operatorname{atan2}
(v_{z,i},v_{x,i}).
\)

其中：

\(\displaystyle
\alpha_0=0.0598428\ {\rm rad}.
\)

---

# 16. SHW09_vtol 升力系数

正常迎角范围：

\(\displaystyle
|\alpha|
\le
\alpha_{\mathrm{stall}}.
\)

其中：

\(\displaystyle
\alpha_{\mathrm{stall}}
=
0.639143\ {\rm rad}.
\)

线性升力模型：

\(\displaystyle
C_L
=
C_{L_\alpha}\alpha.
\)

其中：

\(\displaystyle
C_{L_\alpha}=2.5.
\)

超过正失速角：

\(\displaystyle
C_L
=
C_{L_\alpha}\alpha_{\mathrm{stall}}
+
C_{L,\mathrm{stall}}
(\alpha-\alpha_{\mathrm{stall}}).
\)

其中：

\(\displaystyle
C_{L,\mathrm{stall}}=-2.7.
\)

随后限制：

\(\displaystyle
C_L\ge0.
\)

负失速区域采用对应的对称分段形式。

---

# 17. SHW09_vtol 阻力系数

正常区域采用：

\(\displaystyle
C_D
=
|C_{D_\alpha}\alpha|.
\)

其中：

\(\displaystyle
C_{D_\alpha}=0.4.
\)

失速区继续采用与 Gazebo Classic `LiftDragPlugin` 相同形式的分段线性外推，并最终取：

\(\displaystyle
C_D=|C_D|.
\)

当前使用的 stall slope：

\(\displaystyle
C_{D,\mathrm{stall}}=-0.85.
\)

---

# 18. 单侧机翼气动力

动态压强：

\(\displaystyle
q_i
=
\frac12\rho V_i^2.
\)

阻力方向：

\(\displaystyle
\mathbf e_{D,i}
=
-
\frac{\mathbf v_{LD,i}}
{\|\mathbf v_{LD,i}\|}.
\)

升力方向：

\(\displaystyle
\mathbf e_{L,i}
=
\frac{
\mathbf e_y
\times
\mathbf v_{LD,i}
}{
\|
\mathbf e_y
\times
\mathbf v_{LD,i}
\|
}.
\)

单侧机翼气动力：

\(\displaystyle
\mathbf F_i
=
q_i S_w
\left(
C_{L,i}\mathbf e_{L,i}
+
C_{D,i}\mathbf e_{D,i}
\right).
\)

总主翼气动力：

\(\displaystyle
\mathbf F_{\mathrm{wing}}
=
\mathbf F_L+\mathbf F_R.
\)

总主翼气动力矩：

\(\displaystyle
\mathbf M_{\mathrm{wing}}
=
\mathbf r_L\times\mathbf F_L
+
\mathbf r_R\times\mathbf F_R.
\)

这个模型保留了 GZ / Gazebo Classic 中“左右翼分别受力”的核心结构，同时省略了更复杂的气动细节。

---

# 19. SHW09_vtol 总力与总力矩

总推力：

\(\displaystyle
\mathbf T_B=
\begin{bmatrix}
0\\
0\\
-32.1984u_m^2
\end{bmatrix}.
\)

总控制力矩：

\(\displaystyle
\mathbf M_c
=
\mathbf M_{\mathrm{duct}}
+
\mathbf M_{\mathrm{elevon}}.
\)

主翼气动力矩：

\(\displaystyle
\mathbf M_a
=
\mathbf M_{\mathrm{wing}}
-
K_{DW}\boldsymbol\omega_B.
\)

当前：

\(\displaystyle
K_{DW}=0.025.
\)

总刚体转动方程：

\(\displaystyle
\mathbf I\dot{\boldsymbol\omega}_B
=
\mathbf M_{\mathrm{duct}}
+
\mathbf M_{\mathrm{elevon}}
+
\mathbf M_{\mathrm{wing}}
-
K_{DW}\boldsymbol\omega_B
-
\boldsymbol\omega_B
\times
(\mathbf I\boldsymbol\omega_B).
\)

---

# 20. 地面模型

SIH 保留简化地面接触。

当飞行器进入地面以下且合力仍指向地下时，对悬停类飞行器施加支撑力：

\(\displaystyle
\mathbf F_{\mathrm{ground}}
\approx
-\mathbf F_{\mathrm{total}}.
\)

这样静止状态能够保持：

\(\displaystyle
\mathbf a\approx0.
\)

加速度计输出 specific force：

\(\displaystyle
\mathbf f
=
\mathbf a-\mathbf g.
\)

静止时：

\(\displaystyle
\|\mathbf f\|
\approx
g
\approx9.81\ {\rm m/s^2}.
\)

这保证 EKF2 可以正常完成静止姿态初始化。

当前 `DuctedFan` 和 `DuctedFanTailsitter` 均已加入该地面支撑分支。

---

# 21. 三种机型的模型总结

| 项目 | ductedfan4 | SHC09 | SHW09_vtol |
|---|---:|---:|---:|
| SIH 类型 | DuctedFan | DuctedFan | DuctedFanTailsitter |
| `SIH_VEHICLE_TYPE` | 6 | 6 | 7 |
| 主风扇数 | 1 | 1 | 1 |
| 舵面数 | 4 | 6 | 8 |
| 最大推力 N | 30 | 42.3016 | 32.1984 |
| 质量 kg | 1.56 | 2.10 | 2.05 |
| 主推力 | \(T_{\max}u_m^2\) | \(T_{\max}u_m^2\) | \(T_{\max}u_m^2\) |
| 涵道舵效 | 常数 | 常数 | \(V_e^2\) 连续缩放 |
| 固定翼舵面 | — | — | \(V_a^2\) 连续缩放 |
| 主翼升阻力 | — | — | 左右翼独立简化模型 |
| 执行器时间常数 s | 0.03 | 0.03 | 0.025 |
| 通用角阻尼 | 0 | 0 | 0.025 |

---

# 22. 电脑 SIH 与飞控板 SIH

三种机型共用同一个 `simulator_sih` plant。

## 22.1 电脑运行

```bash
make px4_sitl_sih sihsim_ductedfan4
make px4_sitl_sih sihsim_SHC09
make px4_sitl_sih sihsim_SHW09_vtol
```

对应 airframe：

```text
22018_sihsim_ductedfan4
22019_sihsim_SHC09
22020_sihsim_SHW09_vtol
```

---

## 22.2 飞控板运行

对应 hardware SIH airframe：

```text
1107_rc_ductedfan4_sih.hil
1108_rc_SHC09_sih.hil
1109_rc_SHW09_vtol_sih.hil
```

以 FMUv5 为例：

```bash
make px4_fmu-v5_default
make px4_fmu-v5_default upload
```

然后设置对应：

```text
SYS_AUTOSTART = 1107
```

或：

```text
SYS_AUTOSTART = 1108
```

或：

```text
SYS_AUTOSTART = 1109
```

板载模式中 PX4 控制器、Control Allocator、INDI 和 `simulator_sih` 都运行在飞控 MCU 上。

---

# 23. 主要代码修改

整体代码修改保持较小。

## 23.1 simulator_sih

增加两个 SIH vehicle type：

```text
6 = Ducted Fan
7 = Ducted Fan Tailsitter
```

增加：

```text
VehicleType::DuctedFan
VehicleType::DuctedFanTailsitter
```

---

## 23.2 DuctedFan 参数读取

从已有 Control Allocation 参数直接读取：

```text
CA_ROTOR0_CT
CA_SV_CS_COUNT
CA_SV_CSx_TRQ_R
CA_SV_CSx_TRQ_P
CA_SV_CSx_TRQ_Y
```

因此同一个 airframe 参数同时服务于：

```text
Control Allocator
和
SIH plant
```

---

## 23.3 actuator 数量

`DuctedFan`：

```text
1 motor + 4/6 servos
```

`DuctedFanTailsitter`：

```text
1 motor + 8 servos
```

现有：

```text
NUM_ACTUATORS_MAX = 9
```

刚好覆盖 SHW09_vtol。

---

## 23.4 SIH target

在：

```text
src/modules/simulation/simulator_sih/CMakeLists.txt
```

增加：

```text
ductedfan4
SHC09
SHW09_vtol
```

生成：

```text
sihsim_ductedfan4
sihsim_SHC09
sihsim_SHW09_vtol
```

---

## 23.5 airframe

新增电脑 SIH：

```text
22018_sihsim_ductedfan4
22019_sihsim_SHC09
22020_sihsim_SHW09_vtol
```

新增飞控板 SIH：

```text
1107_rc_ductedfan4_sih.hil
1108_rc_SHC09_sih.hil
1109_rc_SHW09_vtol_sih.hil
```

---

# 24. 当前建模层次

当前模型可以分为三个层次。

## ductedfan4 / SHC09

\(\displaystyle
\boxed{
T=T_{\max}u_m^2
}
\)

\(\displaystyle
\boxed{
\mathbf M_c
=
\sum_i
\mathbf b_i u_i
}
\)

这是最简涵道 SIH。

---

## SHW09_vtol 涵道部分

\(\displaystyle
\boxed{
T=T_{\max}u_m^2
}
\)

\(\displaystyle
\boxed{
\mathbf M_{\mathrm{duct}}
=
k_{\mathrm{duct}}(u_m,V_a)
\sum_{i=1}^{6}
\mathbf b_i u_i
}
\)

---

## SHW09_vtol 固定翼部分

\(\displaystyle
\boxed{
\mathbf M_{\mathrm{elevon}}
=
k_{\mathrm{wing}}(V_a)
(\mathbf b_7u_7+\mathbf b_8u_8)
}
\)

\(\displaystyle
\boxed{
\mathbf F_{\mathrm{wing}}
=
\mathbf F_L+\mathbf F_R
}
\)

\(\displaystyle
\boxed{
\mathbf M_{\mathrm{wing}}
=
\mathbf r_L\times\mathbf F_L
+
\mathbf r_R\times\mathbf F_R
}
\)

因此 SHW09_vtol 的 SIH 保留了：

```text
涵道推力
+ 涵道舵
+ 固定翼舵
+ 主翼升阻力
+ 左右翼气动力矩
+ 刚体动力学
```

同时继续保持远低于 GZ / Gazebo Classic 的模型复杂度。

---

# 25. 建模原则总结

当前 SIH 采用以下原则：

1. `ductedfan4` 和 `SHC09` 使用统一的 `DuctedFan` 简化模型。
2. `SHW09_vtol` 使用独立的 `DuctedFanTailsitter` 简化模型。
3. Control Allocator 继续负责“控制器认为的 effectiveness”。
4. SIH 负责“根据执行器与流场计算实际力和力矩”。
5. SHW09_vtol 的 `DF_FW_CS_GAIN` 留在 CA17，而 plant 使用连续的 \(V_e^2\) / \(V_a^2\) 缩放。
6. GZ / Gazebo Classic 作为高保真和长期验证参考。
7. SIH 只保留对完整飞控逻辑最关键的动力学。
8. 当前模型尤其适合验证 PID、INDI、PCA/LPCA、Control Allocation、VTOL 状态机和板载 SIH 的完整闭环行为。

最终可以将三种机型概括为两组核心方程：

### 涵道悬停类

\(\displaystyle
\boxed{
T=T_{\max}u_m^2,\qquad
\mathbf M_c=\sum_i\mathbf b_i u_i
}
\)

### 涵道尾座式 VTOL

\(\displaystyle
\boxed{
T=T_{\max}u_m^2
}
\)

\(\displaystyle
\boxed{
\mathbf M_c
=
k_{\mathrm{duct}}(u_m,V_a)
\sum_{i=1}^{6}\mathbf b_i u_i
+
k_{\mathrm{wing}}(V_a)
\sum_{i=7}^{8}\mathbf b_i u_i
}
\)

再叠加：

\(\displaystyle
\boxed{
\mathbf F_{\mathrm{wing}},
\quad
\mathbf M_{\mathrm{wing}}
}
\)

进入统一 6-DOF 刚体动力学。

这就是当前三种涵道飞行器 SIH 的主要建模结构。
