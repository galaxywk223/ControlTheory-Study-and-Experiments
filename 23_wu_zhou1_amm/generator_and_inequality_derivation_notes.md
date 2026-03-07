# 生成元与不等式推导笔记

本节整理生成元计算、供给率嵌入、触发瞬间条件以及最终可转化为 LMI 的两类矩阵不等式。

## 4. 生成元与不等式

### 4.1 闭环增广动力学的块表示

回忆增广状态

$$
\xi = \operatorname{col}(x,\tilde x,\tilde w,e_x,e_w) \in \mathbb R^{n_\xi},
\qquad
e = \operatorname{col}(e_x,e_w) \in \mathbb R^{n_e},
\qquad
\tau = t - t_k .
$$

#### 4.1.1 区间内动力学

当 $t \in [t_k, t_{k+1})$ 时，控制律取

$$
u = K_i (x + e_x) + K_{w,i} (\hat w + e_w),
\qquad
\hat w = \tilde w + w .
$$

按增广状态逐块整理后，区间内动力学可统一写成

$$
\dot \xi = F_i \xi + G_i e + H_i d,
\qquad
\dot e = F^{(e)}_i \xi + G^{(e)}_i e + H^{(e)}_i d,
\tag{4.1}
$$

其中外部信号堆叠为

$$
d := \operatorname{col}(w,\dot w),
\qquad
\|\dot w\| \le \rho .
\tag{4.2}
$$

这里 $F_i,G_i,H_i,F^{(e)}_i,G^{(e)}_i,H^{(e)}_i$ 由
$(A_i,B_i,E_i,C_i)$ 与
$(K_i,K_{w,i},L_i,K_i^{\rm obs},\alpha_i)$ 唯一决定。

若需要显式展开，可写成

$$
\begin{aligned}
\dot x &= (A_i + B_i K_i)x + B_i K_{w,i}\tilde w + B_i K_i e_x + B_i K_{w,i} e_w + E_i w, \\
\dot{\tilde x} &= (A_i - L_i C_i)\tilde x + E_i \tilde w - E_i w, \\
\dot{\tilde w} &= -\alpha_i \tilde w + K_i^{\rm obs} C_i \tilde x - \dot w, \\
\dot e_x &= -\dot x, \\
\dot e_w &= -\dot{\hat w} = \alpha_i \tilde w - K_i^{\rm obs} C_i \tilde x + \dot w .
\end{aligned}
$$

将右端按 $(\xi,e,d)$ 线性分组，即可得到式 (4.1) 中的块矩阵。

#### 4.1.2 性能输出

性能输出满足

$$
z = C_{z,i} x + D_{z,i} u .
$$

代入控制律得

$$
z
= \underbrace{(C_{z,i}+D_{z,i}K_i)}_{C_{x,i}} x
+ \underbrace{D_{z,i}K_{w,i}}_{C_{\hat w,i}} \hat w
+ \underbrace{D_{z,i}[K_i \ \ 0]}_{C_{e,i}} e .
\tag{4.3}
$$

又由于 $\hat w = \tilde w + w$，可进一步写成

$$
z = C_\xi^i \xi + C_e^i e + C_d^i d,
\tag{4.4}
$$

其中

$$
C_\xi^i =
\begin{bmatrix}
C_{x,i} & 0 & C_{\hat w,i} & 0 & 0
\end{bmatrix},
\qquad
C_e^i = C_{e,i},
\qquad
C_d^i =
\begin{bmatrix}
C_{\hat w,i} & 0
\end{bmatrix}.
$$

### 4.2 LKF 导数与马尔科夫生成元

回忆第 3 节中的 Lyapunov-Krasovskii 泛函

$$
V_i(\xi,\tau)
= \xi^\top P_i \xi
+ e^\top R_i e
+ 2 \xi^\top N_i e
+ \tau e^\top W_i e
+ \int_{t_k}^{t} \zeta^\top Q_i \zeta \, d\theta,
\tag{3.1 revisited}
$$

其中

$$
\zeta = \operatorname{col}(x,\hat w).
$$

区间内导数可写为

$$
\begin{aligned}
\dot V_i
= {} & 2\xi^\top P_i \dot \xi
+ 2\dot \xi^\top N_i e
+ 2\xi^\top N_i \dot e \\
& + 2 e^\top R_i \dot e
+ \frac{d}{dt}\!\left(\tau e^\top W_i e\right)
+ \zeta^\top Q_i \zeta .
\end{aligned}
\tag{4.5}
$$

将式 (4.1) 代入后，可整理成关于 $(\xi,e,d)$ 的二次型

$$
\dot V_i
=
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix}^{\!\top}
\mathcal M^{\rm flow}_i
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix}
+ \zeta^\top Q_i \zeta,
\tag{4.6}
$$

其中 $\mathcal M^{\rm flow}_i$ 的各块线性依赖于
$(P_i,R_i,N_i,W_i)$ 与
$(F_i,G_i,H_i,F^{(e)}_i,G^{(e)}_i,H^{(e)}_i)$。

#### 积分项上界化

记区间平均量

$$
\bar \zeta := \frac{1}{\tau} \int_{t_k}^{t} \zeta(\theta)\, d\theta,
\qquad \tau > 0,
$$

当 $\tau=0$ 时取 $\bar\zeta=0$。

Jensen 不等式给出

$$
\int_{t_k}^{t} \zeta^\top Q_i \zeta \, d\theta
\ge
\tau \bar\zeta^\top Q_i \bar\zeta .
$$

进一步利用 Wirtinger 或 PII 技术，并引入自由矩阵 $\Xi_i$，可得更紧的下界

$$
\int_{t_k}^{t} \zeta^\top Q_i \zeta \, d\theta
\ge
\begin{bmatrix}
\zeta(t) \\ \bar\zeta
\end{bmatrix}^{\!\top}
\Omega_i(Q_i)
\begin{bmatrix}
\zeta(t) \\ \bar\zeta
\end{bmatrix},
\tag{4.7}
$$

其中 $\Omega_i(Q_i)$ 线性依赖于 $Q_i$ 与 $\Xi_i$。

最终可进一步写成关于 $(\xi,e)$ 的二次型下界

$$
\int_{t_k}^{t} \zeta^\top Q_i \zeta \, d\theta
\ge
\begin{bmatrix}
\xi \\ e
\end{bmatrix}^{\!\top}
\Theta_i(Q_i,\Xi_i)
\begin{bmatrix}
\xi \\ e
\end{bmatrix}.
\tag{4.8}
$$

#### 马尔科夫生成元

区间内的马尔科夫生成元写成

$$
\mathcal L V(\xi,e,i)
= \dot V_i(\xi,e,\tau)
+ \sum_{j\ne i} q_{ij}\big(V_j(\xi,e,\tau)-V_i(\xi,e,\tau)\big).
\tag{4.9}
$$

注意 $V_j - V_i$ 只改变索引，因此

$$
V_j - V_i
=
\begin{bmatrix}
\xi \\ e
\end{bmatrix}^{\!\top}
\Delta_{ji}(\tau)
\begin{bmatrix}
\xi \\ e
\end{bmatrix},
\tag{4.10}
$$

其中

$$
\Delta_{ji}(\tau)
=
\begin{bmatrix}
\Delta P_{ji} & \Delta N_{ji} \\
\Delta N_{ji}^\top & \Delta R_{ji} + \tau \Delta W_{ji}
\end{bmatrix},
$$

并记 $\Delta P_{ji} := P_j - P_i$，其余类似。

### 4.3 供给率嵌入与区间内主不等式

扩展耗散性供给率记为

$$
s_i(z,w)
=
\begin{bmatrix}
z \\ w
\end{bmatrix}^{\!\top}
\Pi_i
\begin{bmatrix}
z \\ w
\end{bmatrix}.
$$

结合式 (4.4)，可写成

$$
s_i(z,w)
=
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix}^{\!\top}
\mathcal S_i(\Pi_i)
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix},
\tag{4.11}
$$

其中

$$
\mathcal S_i(\Pi_i)
=
\begin{bmatrix}
{C_\xi^i}^\top \\
{C_e^i}^\top \\
{C_d^i}^\top
\end{bmatrix}
\Pi_i
\begin{bmatrix}
C_\xi^i & C_e^i & C_d^i
\end{bmatrix}.
$$

区间内的期望衰减条件取为

$$
\mathbb E\big[\mathcal L V(\xi,e,\theta(t)) + s_{\theta(t)}(z,w)\big] \le 0.
\tag{4.12}
$$

将式 (4.6)、(4.8)、(4.10)、(4.11) 合并，可得对任意 $(\xi,e,d)$

$$
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix}^{\!\top}
\Bigg(
\mathcal M^{\rm flow}_i
+ \mathcal S_i(\Pi_i)
+ \begin{bmatrix} I & 0 \\ 0 & I \\ 0 & 0 \end{bmatrix}
\Theta_i
\begin{bmatrix} I & 0 & 0 \\ 0 & I & 0 \end{bmatrix}
+ \sum_{j\ne i} q_{ij}
\begin{bmatrix} I & 0 \\ 0 & I \\ 0 & 0 \end{bmatrix}
\Delta_{ji}(\tau)
\begin{bmatrix} I & 0 & 0 \\ 0 & I & 0 \end{bmatrix}
\Bigg)
\begin{bmatrix}
\xi \\ e \\ d
\end{bmatrix}
\le 0.
\tag{4.13}
$$

这就是区间内主不等式。它对待设计变量
$(P_i,R_i,N_i,W_i,Q_i,K_i,K_{w,i},L_i,K_i^{\rm obs},\alpha_i,\Pi_i)$
保持线性依赖。

### 4.4 触发瞬间不等式与 S-程序处理

触发判据写为

$$
g_i(e,x,\hat w)
:=
e^\top \Phi_i e
-
\begin{bmatrix}
x \\ \hat w
\end{bmatrix}^{\!\top}
\Psi_i
\begin{bmatrix}
x \\ \hat w
\end{bmatrix}
\ge 0,
\qquad
\tau \ge h_{\min}.
\tag{4.14}
$$

触发瞬间的能量增量定义为

$$
\Delta V
:=
V_{i^+}(\xi^+,0)-V_{i^-}(\xi,\tau).
\tag{4.15}
$$

若要求触发不增能量，则有蕴含关系

$$
g_i \ge 0 \Longrightarrow \Delta V \le 0.
\tag{4.16}
$$

利用 S-程序，可将其转化为充分条件：存在标量 $\sigma_i \ge 0$ 使得

$$
\Delta V + \sigma_i g_i \le 0.
\tag{4.17}
$$

将 $(x,\hat w)$ 统一写成 $(\xi,e)$ 的线性组合后，可整理为

$$
\begin{bmatrix}
\xi \\ e
\end{bmatrix}^{\!\top}
\Big(
\mathcal M^{\rm jump}_i(\tau)
+ \sigma_i \mathcal G_i
\Big)
\begin{bmatrix}
\xi \\ e
\end{bmatrix}
\le 0,
\tag{4.18}
$$

其中

$$
\mathcal M^{\rm jump}_i(\tau)
=
\begin{bmatrix}
\mathcal J_i^\top P_{i^+}\mathcal J_i - P_{i^-} & -N_{i^-} \\
-N_{i^-}^\top & -R_{i^-} - \tau W_{i^-}
\end{bmatrix},
$$

而 $\mathcal G_i$ 是将触发条件改写到 $(\xi,e)$ 坐标下后的等效矩阵。

### 4.5 可转 LMI 的两类矩阵不等式

综合区间内演化与触发瞬间条件，可得到后续 LMI 设计所需的两类矩阵不等式。

#### (I) 区间内条件

存在矩阵
$P_i \succ 0$、$R_i \succeq 0$、$W_i \succeq 0$、$Q_i \succeq 0$
及相应自由矩阵，使得

$$
\mathcal H^{\rm flow}_i(\text{vars})
:=
\mathcal M^{\rm flow}_i
+ \mathcal S_i(\Pi_i)
+ \Theta_i
+ \sum_{j\ne i} q_{ij}\Delta_{ji}(\tau)
\preceq 0.
\tag{4.19}
$$

#### (II) 触发瞬间条件

存在 $\sigma_i \ge 0$ 使得

$$
\mathcal H^{\rm jump}_i(\text{vars})
:=
\mathcal M^{\rm jump}_i(\tau)
+ \sigma_i \mathcal G_i
\preceq 0.
\tag{4.20}
$$

若不希望条件显式依赖于 $\tau$，可以去掉其中与 $\tau$ 相关的负半定项以得到保守但简洁的 LMI；若系统存在采样上界，也可以在端点上分别检查。

### 4.6 小结

1. 将 LKF 导数写成了关于 $(\xi,e,d)$ 的二次型。
2. 使用 Jensen / Wirtinger / PII 技术给出积分项的可线性化下界。
3. 将马尔科夫跳变项写成模式差分矩阵 $\Delta_{ji}(\tau)$。
4. 将扩展耗散性供给率嵌入为二次型矩阵 $\mathcal S_i(\Pi_i)$。
5. 用 S-程序把触发瞬间的蕴含条件转写为矩阵不等式。
6. 最终得到区间内与触发瞬间两类可转为 LMI 的条件，供下一节继续做变量替换和增益恢复。
