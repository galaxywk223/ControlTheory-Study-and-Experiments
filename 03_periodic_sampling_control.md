# 线性时不变系统周期采样控制与稳定性分析笔记

## 系统方程

考虑如下线性时不变系统：

```math
\dot{x}(t)=Ax(t)+Bu(t). \qquad \text{(1)}
```

控制器采用周期采样状态反馈形式：

```math
u(t)=Kx(t_k), \qquad t\in[t_k,t_{k+1}), \qquad \text{(2)}
```

其中

```math
t_k = k h,
```

$h$ 为采样周期。

## 问题提出

对于给定的线性时不变系统 (1)，希望设计基于周期采样 $h$ 的状态反馈控制器 (2)，使闭环系统最终渐近稳定。

令

```math
\tau(t)=t-t_k, \qquad \tau(t)\in[0,h), \qquad \dot{\tau}(t)=1,
```

则系统可以改写为时滞形式

```math
\dot{x}(t)=Ax(t)+BKx(t-\tau(t)). \qquad \text{(3)}
```

## 稳定性分析

### 1. Lyapunov-Krasovskii 泛函构建

构造如下 LKF：

```math
V\left(t,x_t,\dot{x}_t\right)
=
V_S\left(t,x_t,\dot{x}_t\right)+V_X\left(t,x_t\right), \qquad \text{(4)}
```

其中

```math
V_S\left(t,x_t,\dot{x}_t\right)
=
x^T(t)Px(t)
+ (h-\tau(t))\int_{t-\tau(t)}^t e^{2\alpha(s-t)}\dot{x}^T(s)U\dot{x}(s)\,ds, \qquad \text{(5)}
```

```math
V_X
=
(h-\tau(t))\xi^T(t)
\begin{pmatrix}
\frac{X+X^T}{2} & -X+X_1 \\
* & -X_1-X_1^T+\frac{X+X^T}{2}
\end{pmatrix}
\xi(t), \qquad \text{(6)}
```

以及

```math
\xi^T(t)=\begin{bmatrix}x^T(t) & x^T(t-\tau(t))\end{bmatrix}. \qquad \text{(7)}
```

对式 (4) 求导，可得到

```math
\dot V\left(t,x_t,\dot{x}_t\right)
=
\dot V_S\left(t,x_t,\dot{x}_t\right)+\dot V_X\left(t,x_t\right). \qquad \text{(8)}
```

再将积分项导数通过莱布尼茨积分法则展开，并整理成关于当前状态、延迟状态、导数项以及辅助变量的二次型表达。

### 2. Jensen 不等式与自由权矩阵

令

```math
v_1=\frac{1}{\tau(t)}\int_{t-\tau(t)}^t \dot{x}(s)\,ds,
```

利用 Jensen 不等式有

```math
\int_{t-\tau(t)}^t \dot{x}^T(s)U\dot{x}(s)\,ds
\ge
\tau(t)v_1^TUv_1. \qquad \text{(10)}
```

再由系统方程与微积分基本定理，可构造两个恒等式约束：

```math
0=
2\left(x^T(t)P_2^T+\dot{x}^T(t)P_3^T\right)
\left(Ax(t)+BKx(t-\tau(t))-\dot{x}(t)\right), \qquad \text{(11)}
```

```math
\int_{t-\tau(t)}^t \dot{x}(s)\,ds=x(t)-x(t-\tau(t)). \qquad \text{(12)}
```

进一步引入自由权矩阵后，可将

```math
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)
```

整理为一个关于增广向量

```math
\eta^T(t)=\begin{bmatrix}
x^T(t) & \dot{x}^T(t) & x^T(t-\tau(t)) & v_1^T
\end{bmatrix}
```

的二次型：

```math
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)
\le
\eta^T(t)\Psi_S\eta(t). \qquad \text{(14)}
```

若 $\Psi_S \prec 0$，则有

```math
\dot V\left(t,x_t,\dot{x}_t\right)+2\alpha V\left(t,x_t,\dot{x}_t\right)<0. \qquad \text{(16)}
```

### 3. 指数稳定性引理

若存在正数 $\beta,\delta$ 和函数

```math
V:\mathbb{R}\times W \times L_2[-h,0]\to\mathbb{R},
```

使得

```math
\beta |\phi(0)|^2 \le V(t,\phi,\dot\phi)\le \delta |\phi|_W^2, \qquad \text{(17)}
```

并且沿系统轨迹满足

```math
\dot{\bar V}(t)+2\alpha \bar V(t)\le 0,
```

则系统以衰减率 $\alpha$ 指数稳定。

因此，本问题的关键就是保证：

1. $\dot V + 2\alpha V < 0$；
2. 泛函 $V$ 同时满足上下界约束。

### 4. $V$ 的下界保证

已知

```math
V\left(t,x_t,\dot{x}_t\right)
=
x^T(t)Px(t)
+ (h-\tau(t))\int_{t-\tau(t)}^t e^{2\alpha(s-t)}\dot{x}^T(s)U\dot{x}(s)\,ds
+ (h-\tau(t))\xi^T(t)\Xi\xi(t), \qquad \text{(19)}
```

其中

```math
\Xi=
\begin{pmatrix}
\frac{X+X^T}{2} & -X+X_1 \\
* & -X_1-X_1^T+\frac{X+X^T}{2}
\end{pmatrix}.
```

由于积分项非负，可得

```math
V\left(t,x_t,\dot{x}_t\right)
\ge
x^T(t)Px(t)+(h-\tau(t))\xi^T(t)\Xi\xi(t). \qquad \text{(20)}
```

进一步定义

```math
\Xi(h)=
\begin{bmatrix}
P+h\frac{X+X^T}{2} & hX_1-hX \\
* & -hX_1-hX_1^T+h\frac{X+X^T}{2}
\end{bmatrix}. \qquad \text{(21)}
```

若存在 $\beta>0$ 使得

```math
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
\end{bmatrix}, \qquad \text{(23)}
```

则可推出

```math
\beta |\phi(0)|^2 \le \bar V(t). \qquad \text{(25)}
```

### 5. 结论

只要同时保证：

- 端点上的矩阵不等式成立；
- LKF 满足上下界；
- 导数满足衰减条件；

则系统在周期采样控制下指数稳定。

## 求解 LMI

### 1. 端点 LMI

由于关键矩阵不等式关于 $\tau\in[0,h]$ 呈仿射形式，因此有

```math
F(\tau)\prec 0,\ \forall \tau\in[0,h]
\Longleftrightarrow
F(0)\prec 0 \text{ 且 } F(h)\prec 0. \qquad \text{(35)}
```

这意味着只需分别在端点 $\tau=0$ 和 $\tau=h$ 检查 LMI，即可推出区间内全局成立。

于是，原本与 $\tau$ 相关的稳定性条件，可以等价地转化为两个端点矩阵不等式。

### 2. 解耦处理

原始矩阵中存在耦合项

```math
\Phi_{13}=P_2^TBK+Y_1^T-T,
\qquad
\Phi_{23}=P_3^TBK+Y_2^T,
```

其中 $P_2,P_3,K$ 都是待求变量，无法直接作为单个 SDP 问题求解。

因此，令

```math
P_2=\gamma_2 I, \qquad P_3=\gamma_3 I, \qquad \text{(43)}
```

则有

```math
\Phi_{13}=\gamma_2 BK+Y_1^T-T,
\qquad
\Phi_{23}=\gamma_3 BK+Y_2^T. \qquad \text{(44)}
```

这样可以减小变量耦合难度，便于用 LMI 工具直接求解。

### 3. 求解结果

取衰减率

```math
\alpha = 0.05,
```

采样周期

```math
h = 0.125,
```

取反馈增益

```math
K=
\begin{bmatrix}
-0.3399 & -0.0263 \\
-0.0628 & -1.2527
\end{bmatrix}. \qquad \text{(45)}
```

## 数值结果

### 1. 参数设置

取初始条件

```math
x(0)=
\begin{bmatrix}
1 \\
-1
\end{bmatrix},
```

系统参数取为

```math
A=
\begin{pmatrix}
-0.479908 & -3.81625 \\
5.1546 & 14.4723
\end{pmatrix},
\qquad
B=\operatorname{diag}(5.8705212,15.50107).
```

控制器采用周期采样律

```math
u(t)=Kx(t_k), \qquad h=0.125\text{ s}.
```

### 2. 采样闭环矩阵

对采样周期 $h$ 做零阶保持离散化，可得到离散闭环矩阵

```math
\Phi_h=A_d+B_dK=
\begin{bmatrix}
0.485077 & -0.156820 \\
1.212949 & -0.969899
\end{bmatrix}.
```

其特征值为

```math
0.339846,\qquad -0.824669,
```

因此谱半径为

```math
\rho(\Phi_h)=0.824669<1.
```

这表明采样闭环系统在离散时刻上是指数收敛的。

### 3. 状态响应

连续状态轨迹与采样时刻状态如下图所示。

![连续状态与采样状态结果](figures/03_periodic_sampling/continuous_sampled_states.png)

可以看到：

- 连续状态 $x_1(t),x_2(t)$ 都逐步衰减到原点；
- 采样点 $x_1[k],x_2[k]$ 与连续轨迹保持一致的衰减趋势；
- 第二维状态的振荡更明显，但包络整体单调减小。

在终点时刻，采样状态约为

```math
x(6)\approx
\begin{bmatrix}
-2.4852\times 10^{-5} \\
-2.0756\times 10^{-4}
\end{bmatrix},
```

说明系统状态已经非常接近原点。

### 4. 控制输入

零阶保持控制输入如下图所示。

![零阶保持控制输入结果](figures/03_periodic_sampling/zoh_control_signals.png)

控制输入在每个采样区间内保持常值，并随着状态衰减而逐步趋于零，这与周期采样控制器的结构一致。

## 小结

这一组结果表明，在给定采样周期 $h=0.125$ 下，闭环离散矩阵的谱半径小于 1，连续状态与采样状态都逐步衰减到原点，零阶保持控制输入也保持有界并同步收敛。
