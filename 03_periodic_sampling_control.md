# 线性时不变系统周期采样控制与稳定性分析笔记

## 系统方程

考虑如下线性时不变系统：

$$
\dot{x}(t)=Ax(t)+Bu(t). \tag{1}
$$

控制器采用周期采样状态反馈形式：

$$
u(t)=Kx(t_k), \qquad t\in[t_k,t_{k+1}), \tag{2}
$$

其中

$$
t_k = k h,
$$

$h$ 为采样周期。

## 问题提出

对于给定的线性时不变系统 (1)，希望设计基于周期采样 $h$ 的状态反馈控制器 (2)，使闭环系统最终渐近稳定。

令

$$
\tau(t)=t-t_k, \qquad \tau(t)\in[0,h), \qquad \dot{\tau}(t)=1,
$$

则系统可以改写为时滞形式

$$
\dot{x}(t)=Ax(t)+BKx(t-\tau(t)). \tag{3}
$$

## 稳定性分析

### 1. Lyapunov-Krasovskii 泛函构建

构造如下 LKF：

$$
V\left(t,x_t,\dot{x}_t\right)
=
V_S\left(t,x_t,\dot{x}_t\right)+V_X\left(t,x_t\right), \tag{4}
$$

其中

$$
V_S\left(t,x_t,\dot{x}_t\right)
=
x^T(t)Px(t)
+ (h-\tau(t))\int_{t-\tau(t)}^t e^{2\alpha(s-t)}\dot{x}^T(s)U\dot{x}(s)\,ds, \tag{5}
$$

$$
V_X
=
(h-\tau(t))\xi^T(t)
\begin{pmatrix}
\frac{X+X^T}{2} & -X+X_1 \\
* & -X_1-X_1^T+\frac{X+X^T}{2}
\end{pmatrix}
\xi(t), \tag{6}
$$

以及

$$
\xi^T(t)=\begin{bmatrix}x^T(t) & x^T(t-\tau(t))\end{bmatrix}. \tag{7}
$$

对式 (4) 求导，可得到

$$
\dot V\left(t,x_t,\dot{x}_t\right)
=
\dot V_S\left(t,x_t,\dot{x}_t\right)+\dot V_X\left(t,x_t\right). \tag{8}
$$

再将积分项导数通过莱布尼茨积分法则展开，并整理成关于当前状态、延迟状态、导数项以及辅助变量的二次型表达。

### 2. Jensen 不等式与自由权矩阵

令

$$
v_1=\frac{1}{\tau(t)}\int_{t-\tau(t)}^t \dot{x}(s)\,ds,
$$

利用 Jensen 不等式有

$$
\int_{t-\tau(t)}^t \dot{x}^T(s)U\dot{x}(s)\,ds
\ge
\tau(t)v_1^TUv_1. \tag{10}
$$

再由系统方程与微积分基本定理，可构造两个恒等式约束：

$$
0=
2\left(x^T(t)P_2^T+\dot{x}^T(t)P_3^T\right)
\left(Ax(t)+BKx(t-\tau(t))-\dot{x}(t)\right), \tag{11}
$$

$$
\int_{t-\tau(t)}^t \dot{x}(s)\,ds=x(t)-x(t-\tau(t)). \tag{12}
$$

进一步引入自由权矩阵后，可将

$$
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)
$$

整理为一个关于增广向量

$$
\eta^T(t)=\begin{bmatrix}
x^T(t) & \dot{x}^T(t) & x^T(t-\tau(t)) & v_1^T
\end{bmatrix}
$$

的二次型：

$$
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)
\le
\eta^T(t)\Psi_S\eta(t). \tag{14}
$$

若 $\Psi_S \prec 0$，则有

$$
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)<0. \tag{16}
$$

### 3. 指数稳定性引理

若存在正数 $\beta,\delta$ 和函数

$$
V:\mathbb{R}\times W \times L_2[-h,0]\to\mathbb{R},
$$

使得

$$
\beta |\phi(0)|^2 \le V(t,\phi,\dot\phi)\le \delta |\phi|_W^2, \tag{17}
$$

并且沿系统轨迹满足

$$
\dot{\bar V}(t)+2\alpha \bar V(t)\le 0,
$$

则系统以衰减率 $\alpha$ 指数稳定。

因此，本问题的关键就是保证：

1. $\dot V + 2\alpha V < 0$；
2. 泛函 $V$ 同时满足上下界约束。

### 4. $V$ 的下界保证

已知

$$
V\left(t,x_t,\dot{x}_t\right)
=
x^T(t)Px(t)
+ (h-\tau(t))\int_{t-\tau(t)}^t e^{2\alpha(s-t)}\dot{x}^T(s)U\dot{x}(s)\,ds
+ (h-\tau(t))\xi^T(t)\Xi\xi(t), \tag{19}
$$

其中

$$
\Xi=
\begin{pmatrix}
\frac{X+X^T}{2} & -X+X_1 \\
* & -X_1-X_1^T+\frac{X+X^T}{2}
\end{pmatrix}.
$$

由于积分项非负，可得

$$
V\left(t,x_t,\dot{x}_t\right)
\ge
x^T(t)Px(t)+(h-\tau(t))\xi^T(t)\Xi\xi(t). \tag{20}
$$

进一步定义

$$
\Xi(h)=
\begin{bmatrix}
P+h\frac{X+X^T}{2} & hX_1-hX \\
* & -hX_1-hX_1^T+h\frac{X+X^T}{2}
\end{bmatrix}. \tag{21}
$$

若存在 $\beta>0$ 使得

$$
\Xi(h)\succ \beta I_{2n},
\qquad
\Xi(0)=
\begin{bmatrix}
P & 0 \\
0 & 0
\end{bmatrix}
\succeq
\begin{bmatrix}
\beta I_n & 0 \\
0 & 0
\end{bmatrix}, \tag{23}
$$

则可推出

$$
\beta |\phi(0)|^2 \le \bar V(t). \tag{25}
$$

### 5. 结论

只要同时保证：

- 端点上的矩阵不等式成立；
- LKF 满足上下界；
- 导数满足衰减条件；

则系统在周期采样控制下指数稳定。

## 求解 LMI

### 1. 端点 LMI

由于关键矩阵不等式关于 $\tau\in[0,h]$ 呈仿射形式，因此有

$$
F(\tau)\prec 0,\ \forall \tau\in[0,h]
\Longleftrightarrow
F(0)\prec 0 \text{ 且 } F(h)\prec 0. \tag{35}
$$

这意味着只需分别在端点 $\tau=0$ 和 $\tau=h$ 检查 LMI，即可推出区间内全局成立。

于是，原本与 $\tau$ 相关的稳定性条件，可以等价地转化为两个端点矩阵不等式。

### 2. 解耦处理

原始矩阵中存在耦合项

$$
\Phi_{13}=P_2^TBK+Y_1^T-T,
\qquad
\Phi_{23}=P_3^TBK+Y_2^T,
$$

其中 $P_2,P_3,K$ 都是待求变量，无法直接作为单个 SDP 问题求解。

因此，令

$$
P_2=\gamma_2 I, \qquad P_3=\gamma_3 I, \tag{43}
$$

则有

$$
\Phi_{13}=\gamma_2 BK+Y_1^T-T,
\qquad
\Phi_{23}=\gamma_3 BK+Y_2^T. \tag{44}
$$

这样可以减小变量耦合难度，便于用 LMI 工具直接求解。

### 3. 求解结果

取衰减率

$$
\alpha = 0.05,
$$

采样周期

$$
h = 0.125,
$$

通过 MATLAB 的 YALMIP + MOSEK 求解可得到

$$
K=
\begin{bmatrix}
-0.3399 & -0.0263 \\
-0.0628 & -1.2527
\end{bmatrix}. \tag{45}
$$

## 数值结果

### 1. 参数设置

取初始条件

$$
x(0)=
\begin{bmatrix}
1 \\
-1
\end{bmatrix},
$$

系统参数取为

$$
A=
\begin{pmatrix}
-0.479908 & -3.81625 \\
5.1546 & 14.4723
\end{pmatrix},
\qquad
B=\operatorname{diag}(5.8705212,15.50107).
$$

控制器采用周期采样律

$$
u(t)=Kx(t_k), \qquad h=0.125\text{ s}.
$$

### 2. 采样闭环矩阵

对采样周期 $h$ 做零阶保持离散化，可得到离散闭环矩阵

$$
\Phi_h=A_d+B_dK=
\begin{bmatrix}
0.485077 & -0.156820 \\
1.212949 & -0.969899
\end{bmatrix}.
$$

其特征值为

$$
0.339846,\qquad -0.824669,
$$

因此谱半径为

$$
\rho(\Phi_h)=0.824669<1.
$$

这表明采样闭环系统在离散时刻上是指数收敛的。

### 3. 状态响应

连续状态轨迹与采样时刻状态如下图所示。

![连续状态与采样状态结果](figures/03_periodic_sampling/continuous_sampled_states.png)

可以看到：

- 连续状态 $x_1(t),x_2(t)$ 都逐步衰减到原点；
- 采样点 $x_1[k],x_2[k]$ 与连续轨迹保持一致的衰减趋势；
- 第二维状态的振荡更明显，但包络整体单调减小。

在终点时刻，采样状态约为

$$
x(6)\approx
\begin{bmatrix}
-2.4852\times 10^{-5} \\
-2.0756\times 10^{-4}
\end{bmatrix},
$$

说明系统状态已经非常接近原点。

### 4. 控制输入

零阶保持控制输入如下图所示。

![零阶保持控制输入结果](figures/03_periodic_sampling/zoh_control_signals.png)

控制输入在每个采样区间内保持常值，并随着状态衰减而逐步趋于零，这与周期采样控制器的结构一致。

## 小结

这份笔记的主线是：

1. 将周期采样控制问题改写为时变时滞系统问题。
2. 构造包含积分项和自由权矩阵的 Lyapunov-Krasovskii 泛函。
3. 用 Jensen 不等式和端点 LMI 技术，将区间稳定性条件转化为有限个矩阵不等式。
4. 通过解耦技巧降低控制器增益与辅助变量之间的耦合难度。
5. 最终通过 LMI 求解控制增益，并通过采样状态响应与控制输入验证闭环系统稳定性。
