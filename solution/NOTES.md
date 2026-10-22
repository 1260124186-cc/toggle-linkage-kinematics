# 领域说明与验算

## 机构模型

折翼压紧工位采用平面四杆机构。机架两端为 `O2=(0,0)` 和 `O4=(d,0)`，曲柄端点 `A` 由曲柄长度和转角直接确定，连杆与摇杆在 `B` 点相接。以 `A->O4` 为基准，`branch=1` 取有向线左侧圆交点，`branch=-1` 取右侧交点。

闭环方程：

```text
a*exp(i*theta2) + b*exp(i*theta3) - c*exp(i*theta4) - d = 0
```

其中 `a,b,c,d` 分别为曲柄、连杆、摇杆和机架长度。圆交点由两圆相交求得，不能根据初始样机固定取一侧。

## 正确值复算

- 速度：对闭环位置方程求时间导，得到关于 `omega3`、`omega4` 的二元一次方程；曲柄角速度是已知输入。
- 加速度：对速度方程继续求导，右端必须包含 `a*omega2^2`、`b*omega3^2` 和 `c*omega4^2` 的向心项，以及曲柄角加速度项。
- 传动角：取连杆正向和摇杆正向所在直线的锐角，范围为 `0` 到 `90` 度。
- 载荷点：`P=O4+ratio*(B-O4)`；固定世界坐标系中的力为 `F=force*(cos(direction),sin(direction))`。
- 载荷力矩：`Q4=(P-O4)_x*F_y-(P-O4)_y*F_x`，曲柄广义力为 `Q_load=Q4*omega4/omega2`。
- 杆重：`dA/dtheta2=crank*(-sin(theta2),cos(theta2))`，`dB/dtheta2=rocker*(omega4/omega2)*(-sin(theta4),cos(theta4))`；各杆质心按给定比例插值，`Q_gravity=sum(m_i*g·dC_i/dtheta2)`，`g=(0,-9.80665)`。
- 回位扭簧：摇杆力矩为 `-stiffness*(theta4-theta_zero)`，曲柄广义力为 `Q_spring=Q4_spring*omega4/omega2`。
- 铰点摩擦：`loss=mu_o2*|omega2|+mu_b*|omega3-omega4|+mu_o4*|omega4|`，为正值。
- 刚体惯性：`Q_inertia=sum[m_i*(a_Ci·dC_i/dtheta2)+I_Ci*alpha_i*(dtheta_i/dtheta2)]`。质心线加速度必须包含切向和向心两部分。
- 曲柄动态力矩：`tau2*omega2+Q_load*omega2+Q_gravity*omega2+Q_spring*omega2-loss-Q_inertia*omega2=0`。

测试端在 `tests/test_outputs.py` 里独立实现圆交点、速度矩阵、二阶导数矩阵、质心导数与质心加速度、载荷/重力/弹簧/惯性广义力和摩擦耗散，不调用待修模块产生期望值。演示工况来自 `environment/workspace/data/example.json`，是机构参数，不是现场实测数据。

## 独立难点

- C1：圆交点与两套装配分支，对应 `geometry.py`。
- C2：闭环一阶速度方程，对应 `kinematics.py` 的速度部分。
- C3：闭环二阶加速度方程和向心项，对应 `kinematics.py` 的加速度部分。
- C4：沿摇杆定位载荷点，用虚功求载荷/重力/弹簧广义力，并用质心线加速度与角加速度求刚体惯性广义力，对应 `loads.py`。
- C5：输入校验、速度和装配可行性检查以及原子写出，对应 `cli.py`。
