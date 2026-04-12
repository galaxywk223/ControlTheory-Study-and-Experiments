# 鲁棒控制笔记

## 基本概念

鲁棒控制研究的是：当系统同时存在内部不确定性和外部扰动时，如何设计反馈控制器，使闭环系统仍然满足给定的稳定性与性能指标。

通俗地说，鲁棒控制就是让系统在“模型不准、参数漂移、外界干扰、测量误差”等不理想条件下，依然能够稳定运行并保持可接受性能。

## 线性时不变系统模型

考虑如下线性时不变系统：

```math
\begin{array}{l}
\dot {x} (t) = A x (t) + B u (t) + D \omega (t), \\
y (t) = C x (t), \\
z (t) = E x (t).
\end{array}
```

其中：

- $x(t)$ 是系统状态。
- $y(t)$ 是测量输出。
- $z(t)$ 是性能输出。
- $u(t)$ 是控制输入。
- $\omega(t) \in L_2[0,\infty)$ 是外部扰动。
- $A,B,C,D,E$ 是适当维数的常矩阵。

## 扰动抑制

### $\mathcal{H}_{\infty}$ 控制

若给定 $\gamma > 0$，并且对任意 $t \ge 0$，在零初始条件下满足

```math
\int_0^t z(s)^T S z(s)\,ds \le \gamma^2 \int_0^t \omega(s)^T \omega(s)\,ds,
```

则称系统具有 $\mathcal{H}_{\infty}$ 扰动抑制水平 $\gamma$，其中 $S$ 为正定对称矩阵。

其物理意义是：输出能量不会超过扰动能量的某个加权放大倍数。$\gamma$ 越小，系统的抗扰性能通常越好。

### $\mathcal{L}_2 - \mathcal{L}_{\infty}$ 控制

若给定 $\gamma > 0$，并且对任意 $t \ge 0$，在零初始条件下满足

```math
\sup_{t \ge 0}\{z(t)^T z(t)\} \le \gamma^2 \int_0^t \omega(s)^T \omega(s)\,ds,
```

则称系统具有 $\mathcal{L}_2 - \mathcal{L}_{\infty}$ 扰动抑制水平 $\gamma$。

这类性能刻画的是：在扰动能量有界的前提下，输出峰值也受到控制。

### 定理 1

对于给定参数 $\gamma > 0$，若存在矩阵 $P$ 使得

```math
E^T E - P < 0,
```

且

```math
\left[
\begin{array}{c c}
He(PA + PBK) & PD \\
* & -\gamma^2 I
\end{array}
\right] < 0,
```

则系统渐近稳定，并且具有 $\mathcal{L}_2 - \mathcal{L}_{\infty}$ 性能指标 $\gamma$。

#### 证明思路

构造李雅普诺夫函数

```math
V(t)=x^T(t)Px(t),
```

再利用牛顿-莱布尼兹公式，把

```math
J(t)=z^T(t)z(t)-\gamma^2\int_0^t \omega(s)^T\omega(s)\,ds
```

与 $\dot V(t)$ 合并，最终得到关于 $\begin{bmatrix}x \\ \omega\end{bmatrix}$ 的二次型上界。若上述矩阵不等式成立，则有 $J(t)\le 0$，从而得到所需性能结论。

## 无源控制

无源性是耗散性的一个特例，强调系统从外界吸收的能量不少于其自身释放的能量。它从输入输出能量关系角度反映系统内部稳定性。

若给定 $\gamma > 0$，并对任意 $t \ge 0$ 满足

```math
2 \int_0^t z^T(s)\omega(s)\,ds \ge -\gamma \int_0^t \omega^T(s)\omega(s)\,ds,
```

则称系统是无源的。

### 定理 2

对于给定参数 $\gamma > 0$，若存在矩阵 $P$ 使得

```math
\left[
\begin{array}{c c}
He(PA + PBK) & PD - E^T \\
* & -\gamma I
\end{array}
\right] < 0,
```

则系统渐近稳定，并且具有无源性能指标 $\gamma$。

#### 证明思路

证明过程与前一节类似，仍然通过李雅普诺夫函数和牛顿-莱布尼兹公式构造积分不等式。只是在这里，构造的性能函数变为

```math
J(t) = -\gamma \int_0^t \omega^T(s)\omega(s)\,ds - 2\int_0^t z^T(s)\omega(s)\,ds,
```

从而把无源条件转写成一个二次型矩阵不等式。

## 耗散控制

耗散理论要求存在一个非负能量函数，使系统的能量损耗始终小于外界供给率。若供给率取特殊形式，耗散控制就会退化为 $\mathcal{H}_{\infty}$ 控制或无源控制。

若给定 $\gamma > 0$，并且对任意 $t \ge 0$ 满足

```math
\int_0^t J(s)\,ds \ge \gamma \int_0^t \omega(s)^T\omega(s)\,ds,
```

则称系统是严格 $(Q,S,R)-\gamma$ 耗散的，其中

```math
J(s)=z^T(s)Qz(s)+2z^T(s)S\omega(s)+\omega^T(s)R\omega(s).
```

### 说明

这个定义包含若干重要特例：

- 当 $\gamma = 0$ 时，退化为普通 $(Q,S,R)$ 耗散。
- 当 $Q=-I, S=0, R=-\gamma^2 I$ 时，退化为 $\mathcal{H}_{\infty}$ 性能。
- 当 $Q=0, S=I, R=0$ 时，退化为严格无源性。

### 定理 3

对于给定参数 $\gamma > 0$，若存在矩阵 $P$ 使得

```math
\left[
\begin{array}{c c}
He(PA + PBK) - E^T Q E & PD - E^T S \\
* & -R + \gamma I
\end{array}
\right] < 0,
```

则系统渐近稳定，并且具有耗散性能。

#### 证明思路

定义辅助函数

```math
\bar J(t)=\gamma\int_0^t \omega^T(s)\omega(s)\,ds-\int_0^t J(s)\,ds,
```

再将其与 $\dot V$ 联立。由于 $z(t)=Ex(t)$，因此供给率可以写成关于 $x$ 和 $\omega$ 的二次型，最终得到矩阵不等式条件。

## 扩展耗散控制

给定实对称矩阵 $\Lambda_1 \ge 0$、$\Lambda_3$、$\Delta \ge 0$ 及实矩阵 $\Lambda_2$，并满足：

- $\left(\|\Lambda_1\|+\|\Lambda_2\|\right)\|\Delta\|=0$；
- 若 $\|\Delta\|\neq 0$，则 $\Lambda_3 < 0$。

若对任意 $t_f \ge 0$，在零初始条件下满足

```math
\sup_{0 \le t \le t_f} z^T(t)\Delta z(t) + \int_0^{t_f} J(s)\,ds \le 0,
```

则称系统是扩展耗散的，其中

```math
J(s)=z^T(s)\Lambda_1 z(s)+2z^T(s)\Lambda_2\omega(s)+\omega^T(s)\Lambda_3\omega(s).
```

### 说明

扩展耗散统一包含以下性能：

- $\mathcal{L}_2 - \mathcal{L}_{\infty}$ 性能；
- $\mathcal{H}_{\infty}$ 性能；
- 无源性能；
- $(Q,S,R)-\gamma$ 耗散性能。

### 定理 4

对于给定参数 $\gamma > 0$，若存在矩阵 $P$ 使得

```math
E^T \Delta E - P < 0,
```

且

```math
\left[
\begin{array}{c c}
He(PA + PBK) + E^T \Lambda_1 E & PD + E^T \Lambda_2 \\
* & \Lambda_3
\end{array}
\right] < 0,
```

则系统渐近稳定，并具有扩展耗散性能。

#### 证明思路

证明分为 $\Delta=0$ 与 $\Delta\neq0$ 两种情况。核心仍然是把

```math
\bar J(t)=z^T(t)\Delta z(t)+\int_0^{t_f}J(s)\,ds
```

与 $\dot V$ 组合成关于 $x$ 和 $\omega$ 的二次型，再由矩阵不等式推出 $\bar J(t)\le 0$。

## 保成本控制

保成本控制也称保性能控制。其目标是在不确定系统鲁棒稳定的同时，使二次型成本函数不超过某个预先给定的上界。

通常成本函数写为

```math
\int_0^\infty \left(x^T(t)Qx(t)+u^T(t)Ru(t)\right)\,dt.
```

若存在反馈控制 $u(t)=Kx(t)$ 使得成本值满足 $J \le J^*$，则称 $J^*$ 为成本上界，$u(t)$ 为保成本控制。

## 参数不确定性

鲁棒控制的核心对象就是系统中的不确定性。常见不确定性形式包括：

- 范数有界不确定性；
- 区间不确定性；
- 凸多面体不确定性。

### 范数有界不确定性

若

```math
A = A + \Delta A, \qquad B = B + \Delta B,
```

且不确定项满足

```math
\left[ \Delta A \quad \Delta B \right] = H F(t)\left[ N_1 \quad N_2 \right],
```

其中 $F^T(t)F(t)\le I$，则称其为范数有界不确定性。

### 区间不确定性

若矩阵元素分别在上下界之间变化，即

```math
\underline{A} \le A \le \overline{A}, \qquad
\underline{B} \le B \le \overline{B},
```

则称为区间不确定性。

通过定义

```math
A_0=\frac{1}{2}(\overline A+\underline A), \qquad
A_1=\frac{1}{2}(\overline A-\underline A),
```

```math
B_0=\frac{1}{2}(\overline B+\underline B), \qquad
B_1=\frac{1}{2}(\overline B-\underline B),
```

可进一步把区间不确定性转化为范数有界不确定性。

### 凸多面体不确定性

若

```math
\left[\begin{array}{c c} A & B \end{array}\right]
=
\sum_{i=1}^{N}\alpha_i
\left[\begin{array}{c c} A_i & B_i \end{array}\right],
\qquad
\sum_{i=1}^{N}\alpha_i=1,
\qquad
\alpha_i \ge 0,
```

则称为凸多面体不确定性。

## 定理 5

对于给定参数 $\gamma > 0$，若存在矩阵 $P$ 使得

```math
\left[
\begin{array}{c c}
He\{P(A+\Delta A)+P(B+\Delta B)KC\}+E^TE & PD \\
* & -\gamma^2 I
\end{array}
\right] < 0,
```

则系统渐近稳定，并且具有给定的 $H_{\infty}$ 性能指标 $\gamma$。

#### 证明思路

对系统轨迹求导可得李雅普诺夫导数的二次型表达，再构造

```math
J=\int_0^t \left(z^T(t)z(t)-\gamma^2\omega^T(t)\omega(t)\right)\,dt,
```

并借助牛顿-莱布尼兹公式，将问题转化为一个关于 $\begin{bmatrix}x \\ \omega\end{bmatrix}$ 的矩阵不等式判定问题。

## 数值结果

### 系统设置

取如下区间不确定系统：

```math
\dot x(t)=A(\rho_1,\rho_2)x(t)+Bu(t)+D\omega(t), \qquad z(t)=Ex(t),
```

其中

```math
A(\rho_1,\rho_2)=
\begin{bmatrix}
0 & 1 \\
1.2+\rho_1 & 0.3+\rho_2
\end{bmatrix},
\qquad
\rho_1 \in [-0.6,0.6], \quad \rho_2 \in [-0.25,0.25],
```

```math
B=D=
\begin{bmatrix}
0 \\
1
\end{bmatrix},
\qquad
E=
\begin{bmatrix}
1 & 0 \\
0 & 0.4
\end{bmatrix}.
```

控制律取为

```math
u(t)=Kx(t),
\qquad
K=
\begin{bmatrix}
-3.9276 & -3.4536
\end{bmatrix}.
```

对应的一个公共李雅普诺夫矩阵可取为

```math
P=
\begin{bmatrix}
8.2419 & 3.9276 \\
3.9276 & 3.4536
\end{bmatrix}.
```

扰动输入取为

```math
\omega(t)=0.8 e^{-0.3t}\sin(2.4t).
```

### 区间顶点判定

先在四个区间顶点和名义点上计算开环谱横坐标、闭环谱横坐标、李雅普诺夫残差最大特征值以及从 $\omega$ 到 $z$ 的频域增益估计值。结果如下表所示：

| 情形    | $\rho_1$ | $\rho_2$ | 开环谱横坐标 | 闭环谱横坐标  | $\lambda_{\max}(A_c^TP+PA_c)$ | $\|G_{\omega z}\|_\infty$ 估计 |
| ----- | --------:| --------:| ------:| -------:| -----------------------------:| ----------------------------:|
| $V_1$ | -0.60    | -0.25    | 0.8000 | -1.7018 | -3.4709                       | 0.3005                       |
| $V_2$ | -0.60    | 0.25     | 1.0970 | -1.4518 | -2.9424                       | 0.3005                       |
| $V_3$ | 0.60     | -0.25    | 1.3669 | -0.8252 | -3.6982                       | 0.4700                       |
| $V_4$ | 0.60     | 0.25     | 1.6445 | -1.4518 | -3.7070                       | 0.4700                       |
| 名义点   | 0.00     | 0.00     | 1.2557 | -1.5768 | -3.6037                       | 0.3666                       |

四个顶点的开环谱横坐标都为正，说明不确定系统在整个区间内都存在明显不稳定趋势。加上同一个反馈增益以后，四个顶点和名义点的闭环谱横坐标全部转为负值，且公共李雅普诺夫残差也保持负定。对这组参数，顶点上的最坏频域增益估计约为

```math
\gamma^\star \approx 0.47 < 0.5.
```

### 谱横坐标扫描结果

对整个不确定区间做网格扫描后，可以直接看到开环与闭环在稳定性上的差别：

![区间不确定系统的开环与闭环谱横坐标](figures/04_robust_control/spectral_abscissa_scan.png)

左图中全部区域都位于正谱横坐标范围，表示开环模型在参数漂移下普遍不稳；右图则全部落在负半平面内，说明该反馈增益在整个扫描区间内都保持闭环稳定。

### 顶点处的抗扰指标

四个区间顶点与名义点上的 $\mathcal{H}_{\infty}$ 增益估计如下图所示：

![区间顶点处的频域增益估计](figures/04_robust_control/vertex_hinf_estimates.png)

图中的虚线给出 $\gamma=0.5$。所有柱形都落在虚线下方，其中较大的两项出现在 $\rho_1=0.6$ 的顶点附近，这也对应了稳定裕度相对较小的那一侧参数区域。

### 时域响应

在零初始状态下，把同一个衰减扰动 $\omega(t)$ 施加到三个代表性参数点 $(\rho_1,\rho_2)=(-0.6,-0.25)$、$(0,0)$、$(0.6,0.25)$，可得到如下状态响应：

![不同不确定参数下的闭环状态响应](figures/04_robust_control/state_response_under_disturbance.png)

三组曲线都保持有界并逐步收敛到零附近。三种代表性情形下的状态峰值分别约为 $0.1587$、$0.1673$、$0.1744$。

对应控制输入如下图所示：

![不同不确定参数下的控制输入](figures/04_robust_control/control_response_under_disturbance.png)

三种情形下控制输入峰值分别约为 $0.7027$、$0.7702$、$0.8513$。随着 $\rho_1,\rho_2$ 取到更不利的右上角顶点，状态与控制幅值都会上升，但闭环仍保持稳定，且扰动消失后输入逐步衰减。
