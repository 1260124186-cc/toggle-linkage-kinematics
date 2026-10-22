# 完成折翼压紧四杆机构的运动与载荷核算

包装线的折翼压紧工位用一台曲柄摇杆机构带动压板。曲柄连续转动，摇杆沿纸盒折翼方向施加压紧力，机架上还有一根回位扭簧。设备组需要复核一整周内各杆位置、摇杆加速度、传动角和曲柄所需扭矩。旧工具只适用于单侧装配、无杆重且铰点理想润滑的样机，换到双分支工位后结果无法直接使用。请保留现有包名和命令入口，把计算补完整。

命令入口固定为：

```bash
python -m toggle_linkage.cli CASE.json REPORT.json
```

输入是 UTF-8 JSON，根对象包含 `mechanism`、`mass_properties`、`return_spring`、`joint_friction`、`input` 和 `load` 六部分。

`mechanism` 包含 `ground_m`、`crank_m`、`coupler_m`、`rocker_m` 和 `branch`。四个杆长均为有限正数；`branch` 只能取 `1` 或 `-1`。机架铰点在 `O2=(0,0)` 和 `O4=(ground_m,0)`。曲柄自由端为 `A`，连杆与摇杆的连接点为 `B`。以 `A` 指向 `O4` 的有向线段为准，`branch=1` 选择该线左侧的圆交点，`branch=-1` 选择右侧交点。连杆正向取 `A->B`，摇杆正向取 `O4->B`。

`mass_properties` 包含 `crank`、`coupler`、`rocker` 三个对象和 `gravity_m_s2`。每个杆对象包含 `mass_kg`、`com_ratio` 与 `inertia_kg_m2`：曲柄和摇杆的质心分别从 `O2`、`O4` 沿该杆正向按 `com_ratio` 定位，连杆质心从 `A` 向 `B` 按 `com_ratio` 定位；`inertia_kg_m2` 是绕过该质心且垂直于机构平面的转动惯量。质量与转动惯量必须为正，质心比例必须大于 0 且不超过 1，重力加速度必须为正。重力方向固定为世界坐标系 `(0,-gravity_m_s2)`。

`return_spring` 包含 `stiffness_n_m_per_rad` 和 `zero_angle_deg`。扭簧作用在摇杆上，对摇杆的力矩为 `-stiffness*(rocker_angle-zero_angle)`，刚度必须非负，零位角必须有限。`joint_friction` 包含 `o2_n_m`、`b_n_m` 和 `o4_n_m`，分别是曲柄与机架、连杆与摇杆、摇杆与机架三个转动铰点的库仑摩擦力矩，三个数都必须非负且有限。

`input.crank_angles_deg` 是非空数组，每个角度均为有限数且满足 `0 <= angle < 360`；`input.crank_speed_rad_s` 是有限非零曲柄角速度，正方向为逆时针；`input.crank_acceleration_rad_s2` 是有限曲柄角加速度。

`load.force_n` 是有限的非负载荷力大小，`direction_deg` 是固定世界坐标系中的方向角，`application_ratio` 表示载荷点沿 `O4->B` 的比例，必须大于 0 且不超过 1。载荷只计入这一条外力，不考虑杆件自重、摩擦和惯性力。

对每个采样角先求闭环位置，再求连杆和摇杆角速度及角加速度。闭环方程使用
`crank*exp(i*crank_angle) + coupler*exp(i*coupler_angle) - rocker*exp(i*rocker_angle) - ground = 0`。
速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出；不要用相邻采样点的差分代替瞬时值。当圆交点不存在、速度方程奇异或参数非法时应拒绝整份计算。

传动角定义为连杆正向与摇杆正向所在直线的锐角，取值范围为 `0` 到 `90` 度。载荷点取 `P = O4 + application_ratio * (B - O4)`，二维叉积按 `cross(r,F)=r_x*F_y-r_y*F_x`。摇杆上的载荷力矩为 `Q4=cross(P-O4,F)`，载荷的曲柄广义力为 `Q_load=Q4*rocker_speed/crank_speed`。

对曲柄角 `θ2` 求各单位置的导数：`dA/dθ2=a*(-sinθ2,cosθ2)`，`dB/dθ2=rocker_m*(rocker_speed/crank_speed)*(-sinθ4,cosθ4)`，连杆质心位置按 `A->B` 的比例插值。各杆重力对曲柄的广义力为 `Q_gravity=Σ m_i*g·(dC_i/dθ2)`，这里 `g=(0,-gravity_m_s2)`。曲柄质心、连杆质心和摇杆质心的线加速度分别由 `A` 点加速度、`B` 点加速度及其质心比例插值得到。

回位扭簧对摇杆的力矩为 `Q4_spring=-stiffness*(rocker_angle-zero_angle)`，其曲柄广义力为 `Q_spring=Q4_spring*rocker_speed/crank_speed`。三个转动铰点的摩擦功率为
`friction_loss=|crank_speed|*o2_n_m+|coupler_speed-rocker_speed|*b_n_m+|rocker_speed|*o4_n_m`，
它是正的能量耗散，不是带符号的驱动力。

机构在给定曲柄角速度和角加速度下的广义惯性力按刚体动能计算：
`Q_inertia=Σ[m_i*(a_Ci·dC_i/dθ2)+I_Ci*alpha_i*(dtheta_i/dθ2)]`，
其中曲柄的 `dtheta/dθ2=1`，连杆和摇杆分别取 `coupler_speed/crank_speed`、`rocker_speed/crank_speed`。`a_Ci` 必须在世界坐标系中计算，不能只取切向项。

曲柄力矩按瞬时功率平衡求：
`drive_torque*crank_speed + Q_load*crank_speed + Q_gravity*crank_speed + Q_spring*crank_speed - friction_loss - Q_inertia*crank_speed = 0`。
载荷功率、重力功率和弹簧功率分别为对应广义力乘曲柄角速度；`kinetic_power=Q_inertia*crank_speed` 是机构动能变化率；曲柄功率为 `drive_torque*crank_speed`。

输出根对象固定包含 `mechanism`、`samples` 和 `summary`。`mechanism` 包含 `grashof_class`、`shortest_link` 和 `branch`；当曲柄是最短杆、与机架相邻且满足 Grashof 条件 `最短杆+最长杆 <= 另外两杆之和` 时，`grashof_class` 写 `crank-rocker`，否则写 `other`。`shortest_link` 只能写 `ground`、`crank`、`coupler` 或 `rocker`。

`samples` 保持输入角顺序，每项包含：

```text
crank_angle_deg
crank_angle_rad
coupler_angle_deg
rocker_angle_deg
coupler_point_m
load_point_m
coupler_omega_rad_s
rocker_omega_rad_s
coupler_alpha_rad_s2
rocker_alpha_rad_s2
transmission_angle_deg
load_torque_n_m
gravity_generalized_torque_n_m
spring_generalized_torque_n_m
inertial_generalized_torque_n_m
friction_loss_w
drive_torque_n_m
load_power_w
gravity_power_w
spring_power_w
kinetic_power_w
drive_power_w
```

角度输出用度，角速度用 rad/s，角加速度用 rad/s²，点坐标用米。`coupler_point_m` 是 `[B_x,B_y]`，`load_point_m` 是 `[P_x,P_y]`。所有数值保留浮点精度，不为显示提前取整。

`summary` 固定包含 `minimum_transmission_angle_deg`、`maximum_abs_coupler_speed_rad_s`、`maximum_abs_rocker_speed_rad_s`、`maximum_abs_drive_torque_n_m`、`maximum_friction_loss_w` 和 `maximum_abs_kinetic_power_w`，分别从以上采样结果取最小值或最大绝对值。

缺字段、空角度数组、非有限数、非正杆长、质量或转动惯量、非法分支、角度越界、质心比例或载荷比例越界、负弹簧刚度、负铰点摩擦、圆交点不存在或速度矩阵奇异时，命令应以非零状态退出，并且不能生成新的报告，也不能覆盖已有报告。成功路径应先完成整份计算，再原子替换目标文件；如果报告路径的父目录尚不存在，成功路径也应创建它。`/workspace/data/example.json` 是一组演示用机构参数，不是现场测量记录。
