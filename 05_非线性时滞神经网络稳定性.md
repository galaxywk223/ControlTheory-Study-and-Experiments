# 非线性时滞神经网络稳定性笔记

## 非线性系统模型

考虑如下时滞神经网络

```math
\dot {x} (t) = A x (t) + W f (x (t - \tau)).
```

其中：

- $x(t) = [x_1(t)\dots x_n(t)]^T$ 是神经元状态向量。
- $A = -\mathrm{diag}\{a_{1}, a_{2}, \ldots, a_{n}\}$，其中 $a_i > 0$，是自反馈矩阵。
- $W$ 是连接权矩阵。
- $\tau$ 是时滞。
- $f(x(t)) = [f_1(x_1(t)) \cdots f_n(x_n(t))]^T$ 是激活函数向量。

## 激活函数假设

### 假设 1

Lipschitz 条件：存在 $\alpha_i > 0$，$i=1,\dots,n$，使得

```math
\left| f _ {i} (a) - f _ {i} (b) \right|
\leq
\alpha_ {i} | a - b |,
\qquad
\forall a, b \in \mathbb {R}, \ a \neq b.
```

并且

```math
f_i(0)=0.
```

### 假设 2

单调有界斜率条件：存在 $\alpha_i > 0$，使得

```math
0 \leq \frac {f _ {i} (a) - f _ {i} (b)}{a - b} \leq \alpha_ {i},
\qquad
\forall a, b \in \mathbb {R}, \ a \neq b.
```

并且

```math
f_i(0)=0.
```

### 说明

假设 2 比假设 1 更强。假设 1 只要求 Lipschitz 连续性，而假设 2 还要求单调性。

## 稳定性分析

### 定理 1

设假设 1 成立。若存在矩阵 $P > 0$ 和对角矩阵 $Q > 0$，使得

```math
\left[
\begin{array}{c c}
H e (P A) + Q & P W \\
* & - L ^ {- 1} Q L ^ {- 1}
\end{array}
\right] < 0,
```

则该时滞神经网络系统渐近稳定。

#### 证明思路

构造李雅普诺夫泛函

```math
V (t) = x ^ {T} (t) P x (t) + \int_ {t - \tau} ^ {t} x ^ {T} (s) Q x (s) d s.
```

对其求导得

```math
\dot {V} (t)
=
x ^ {T} (t) (A ^ {T} P + P A + Q)
+ f ^ {T} (x (t - \tau)) W ^ {T} P x (t)
+ x ^ {T} (t) P W f(x(t-\tau)).
```

由假设 1 可得

```math
f _ {i} ^ {2} \left(x _ {i}\right) \leq \alpha_ {i} ^ {2} | x _ {i} | ^ {2}.
```

令

```math
L = \mathrm{diag}\bigl(\alpha_1,\alpha_2,\ldots,\alpha_n\bigr).
```

则有

```math
f ^ {T} (x (t - \tau)) L ^ {- 1} Q L ^ {- 1} f (x (t - \tau))
\leq
x ^ {T} (t - \tau) Q x (t - \tau). \qquad \text{(2)}
```

将导数估计与式 (2) 联立可得

```math
\dot {V} (t)
\leq
\left[
\begin{array}{c}
x (t) \\
f (x (t - \tau))
\end{array}
\right] ^ {T}
\left[
\begin{array}{c c}
H e (P A) + Q & P W \\
* & - L ^ {- 1} Q L ^ {- 1}
\end{array}
\right]
\left[
\begin{array}{c}
x (t) \\
f (x (t - \tau))
\end{array}
\right].
```

因此，在矩阵不等式条件下，

```math
\dot {V} (t) < 0,
```

从而可证明系统渐近稳定。

### 定理 2

设假设 1 成立。若存在矩阵 $P > 0$ 和对角矩阵 $Q > 0$，使得

```math
\left[
\begin{array}{c c}
H e (P A) + L Q L & P W \\
* & - Q
\end{array}
\right] < 0,
```

则系统渐近稳定。

#### 证明思路

构造李雅普诺夫泛函

```math
V (t) = x ^ {T} (t) P x (t) + \int_ {t - \tau} ^ {t} f ^ {T} (x (s)) Q f (x (s)) d s.
```

求导可得

```math
\dot {V} (t)
=
x ^ {T} (t) \left(A ^ {T} P + P A\right) x (t)
+ 2 x ^ {T} (t) P W f (x (t - \tau))
+ f ^ {T} (x (t)) Q f (x (t)).
```

由假设 1，

```math
\sum_ {i = 1} ^ {n} q _ {i} f _ {i} ^ {2} \left(x _ {i}\right)
\leq
\sum_ {i = 1} ^ {n} q _ {i} \alpha_ {i} ^ {2} | x _ {i} | ^ {2}.
```

因此，

```math
f ^ {T} (x (t)) Q f (x (t))
\leq
x ^ {T} (t) L Q L x (t). \qquad \text{(4)}
```

将导数估计与式 (4) 联立，可得

```math
\dot {V} (t)
\leq
\left[
\begin{array}{c}
x (t) \\
f (x (t - \tau))
\end{array}
\right] ^ {T}
\left[
\begin{array}{c c}
H e (P A) + L Q L & P W \\
* & - Q
\end{array}
\right]
\left[
\begin{array}{c}
x (t) \\
f (x (t - \tau))
\end{array}
\right].
```

从而系统渐近稳定。

### 定理 3

设假设 2 成立。若存在 $P > 0$、$Q > 0$、$L > 0$ 以及对角矩阵 $R > 0$、$U > 0$，使得

```math
\left[
\begin{array}{c c c}
H e (P A) & R A + U L & P W \\
* & Q - H e (U) & R W \\
* & * & - Q
\end{array}
\right] < 0,
```

则系统渐近稳定。

#### 证明思路

构造李雅普诺夫泛函

```math
V (t)
=
x ^ {T} (t) P x (t)
+ \int_ {t - \tau} ^ {t} f ^ {T} (x (s)) Q f (x (s)) d s
+ 2 \sum_ {i = 1} ^ {n} r _ {i} \int_ {0} ^ {x _ {i} (t)} f _ {i} (s) d s.
```

则有

```math
\dot {V} (t)
=
\left[
\begin{array}{c}
x (t) \\
f (x (t)) \\
f (x (t - \tau))
\end{array}
\right] ^ {T}
\left[
\begin{array}{c c c}
H e (P A) & A ^ {T} R ^ {T} & P W \\
* & Q & R W \\
* & * & - Q
\end{array}
\right]
\left[
\begin{array}{c}
x (t) \\
f (x (t)) \\
f (x (t - \tau))
\end{array}
\right].
```

由假设 2，

```math
0 \leq \frac {f _ {i} \left(x _ {i}\right)}{x _ {i}} \leq \alpha_ {i},
```

可进一步得到

```math
f _ {i} ^ {2} \left(x _ {i}\right) \leq \alpha_ {i} x _ {i} f _ {i} \left(x _ {i}\right).
```

对 $u_i > 0$，

```math
\sum u _ {i} f ^ {2} \left(x _ {i}\right)
\leq
\sum u _ {i} \alpha_ {i} x _ {i} f \left(x _ {i}\right),
```

即

```math
f ^ {T} (x (t)) U f (x (t))
\leq
f ^ {T} (x (t)) U L x (t).
```

因此，

```math
2 \left(f ^ {T} (x (t)) U L x (t) - f ^ {T} (x (t)) U f (x (t))\right) \geq 0. \qquad \text{(6)}
```

将导数估计与式 (6) 联立，即可得到定理 3 中的充分条件矩阵不等式，从而证明系统渐近稳定。

## 数值结果

### 系统设置

取两神经元时滞网络

```math
\dot x(t)=Ax(t)+Wf(x(t-\tau)),
\qquad
f(x)=\tanh(x),
```

其中

```math
A=
\begin{bmatrix}
-1.0 & 0 \\
0 & -0.9
\end{bmatrix},
\qquad
W=
\begin{bmatrix}
0.35 & -0.28 \\
0.22 & 0.31
\end{bmatrix}.
```

由于

```math
0 \le \frac{d}{ds}\tanh(s) \le 1,
```

因此这里可取

```math
L=I_2.
```

历史函数取为常值

```math
x(s)=
\begin{bmatrix}
1.2 \\
-1.0
\end{bmatrix},
\qquad
s \in [-\tau,0].
```

### 三条定理的数值证书

对定理 1，可取

```math
P_1=
\begin{bmatrix}
1.4722 & 0 \\
0 & 3.3453
\end{bmatrix},
\qquad
Q_1=
\begin{bmatrix}
1.6677 & 0 \\
0 & 1.7276
\end{bmatrix}.
```

对定理 2，可取

```math
P_2=
\begin{bmatrix}
0.6477 & 0 \\
0 & 3.4040
\end{bmatrix},
\qquad
Q_2=
\begin{bmatrix}
0.4962 & 0 \\
0 & 3.0585
\end{bmatrix}.
```

对定理 3，可取

```math
P_3=
\begin{bmatrix}
2.5015 & 0 \\
0 & 2.1902
\end{bmatrix},
\qquad
Q_3=
\begin{bmatrix}
2.5932 & 0 \\
0 & 1.4982
\end{bmatrix},
```

```math
R=
\begin{bmatrix}
1.6932 & 0 \\
0 & 0.6653
\end{bmatrix},
\qquad
U=
\begin{bmatrix}
1.8966 & 0 \\
0 & 2.2641
\end{bmatrix}.
```

把它们代入三条定理对应的块矩阵后，最大特征值分别为

| 判定条件 | 最大特征值   |
| ---- | -------:|
| 定理 1 | -0.7917 |
| 定理 2 | -0.1977 |
| 定理 3 | -0.5380 |

三项都严格小于零，说明这组三个判定条件在该网络上都可以直接给出渐近稳定结论。

对应结果如下图所示：

![三条定理对应的 LMI 判定裕度](figures/05_delay_neural_network_stability/lmi_certificate_margins.png)

### 不同时滞下的状态轨迹

在 $\tau=0.2$、$\tau=1.0$、$\tau=2.0$ 三种时滞下，状态轨迹如下：

![不同时滞下的神经网络状态轨迹](figures/05_delay_neural_network_stability/state_trajectories_by_delay.png)

三种时滞下的轨迹都收敛到原点附近。收敛到

```math
\|x(t)\|_2 \le 10^{-2}
```

后不再离开的时间分别约为 $8.458\,\mathrm{s}$、$12.056\,\mathrm{s}$、$17.074\,\mathrm{s}$。时滞增大以后，收敛速度明显下降，但并没有破坏稳定性。

### 时滞扫描结果

进一步在

```math
\tau \in [0,2.2]
```

上做等步长扫描，可以得到时滞与收敛时间之间的关系：

![时滞与收敛时间的关系](figures/05_delay_neural_network_stability/delay_settling_time_scan.png)

这组参数下，随着时滞从 $0$ 增加到 $2.2$，收敛时间从约 $7.762\,\mathrm{s}$ 单调增长到约 $18.090\,\mathrm{s}$。这说明当前网络虽然具备较强的时滞鲁棒性，但时滞仍会显著拖慢衰减过程。

### 相平面轨迹

当 $\tau=2.0$ 时，相平面轨迹如下：

![时滞为 2.0 时的相平面轨迹](figures/05_delay_neural_network_stability/phase_portrait_tau_2_0.png)

轨线从初始历史状态附近逐步卷向原点，和前面的时域轨迹结论一致。
