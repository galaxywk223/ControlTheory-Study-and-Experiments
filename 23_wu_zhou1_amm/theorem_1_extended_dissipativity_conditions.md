# Theorem 1: Extended Dissipativity Conditions

## Theorem Statement

For given real matrices

$$
\Upsilon_{1} \leq 0, \qquad \Upsilon_{2}, \qquad \Upsilon_{3} > 0, \qquad \Upsilon_{4} \geq 0,
$$

satisfying

$$
(\|\Upsilon_{1}\| + \|\Upsilon_{2}\|)\cdot \|\Upsilon_{4}\| = 0,
$$

and scalars

$$
\varepsilon_{1}, \varepsilon_{2}, \gamma > 0, \qquad \delta \geq 0, \qquad d > 0, \qquad \alpha > 0, \qquad \sigma \geq 0,
$$

assume that for any $i \in \bar{\mathbb{N}}$, there exist matrices

$$
P_i > 0, \qquad U > 0, \qquad
\bar{Q} =
\begin{bmatrix}
Q_1 & Q_2 \\
* & Q_3
\end{bmatrix}
> 0,
$$

along with $Y$, $H_m$ $(m = 1,2,3,4,5)$, $G_n$ $(n = 1,2,3,4)$, and

$$
\Lambda_i \geq 0,
$$

such that the following conditions hold.

## Matrix Inequalities

### 1. Main LMIs

$$
\Psi(i)=
\begin{bmatrix}
\Psi_{11}(i) & \Psi_{12}(i) & \Psi_{13}(i) & \Psi_{14} & \Psi_{15}(i) & \Psi_{16} \\
* & \Psi_{22} & \Psi_{23}(i) & 0 & \Psi_{25}(i) & \Psi_{26} \\
* & * & \Psi_{33}(i) & \Psi_{34} & \Psi_{35}(i) & \Psi_{36} \\
* & * & * & \Psi_{44} & 0 & \Psi_{46} \\
* & * & * & * & \Psi_{55} & 0 \\
* & * & * & * & * & \Psi_{66}
\end{bmatrix}
< 0,
$$

$$
\Theta(i)=
\begin{bmatrix}
\Theta_{11}(i) & \Theta_{12}(i) & \Theta_{13}(i) & \Theta_{14} & \Theta_{15}(i) \\
* & \Theta_{22} & \Theta_{23}(i) & \Theta_{24} & \Theta_{25}(i) \\
* & * & \Theta_{33}(i) & \Theta_{34} & \Theta_{35}(i) \\
* & * & * & \Theta_{44} & 0 \\
* & * & * & * & \Theta_{55}
\end{bmatrix}
< 0,
$$

$$
\Phi(i)=
\begin{bmatrix}
\Phi_{11}(i) & \Phi_{12}(i) & \Phi_{13}(i) & \Phi_{14}(i) \\
* & \Phi_{22} & \Phi_{23}(i) & \Phi_{24}(i) \\
* & * & \Phi_{33}(i) & 0 \\
* & * & * & \Phi_{44}
\end{bmatrix}
< 0.
$$

### 2. Positivity Constraints

$$
\Xi(i)=
\begin{bmatrix}
P_i + dS\!\left(\frac{H_1}{2}\right) - \Upsilon_4 & -dH_1 + dH_2 & dH_3 \\
* & -dS\!\left(H_2 - \frac{H_1}{2}\right) & dH_4 \\
* & * & dS\!\left(\frac{H_5}{2}\right)
\end{bmatrix}
> 0,
$$

$$
P_i - \Upsilon_4 > 0.
$$

## Block Definitions

The block entries are given by

$$
\begin{aligned}
\Psi_{11}(i) &=
\sum_{j=1}^N \lambda_{ij}^* P_j
+ 2\alpha P_i
+ S\left(-\frac{H_1}{2}+G_1+Y\overline{A}_i\right)
- Y_1, \\
\Psi_{12}(i) &= P_i + G_2 - Y + \varepsilon_1 \overline{A}_i^T Y^T, \\
\Psi_{13}(i) &= H_1 - H_2 + G_3 - G_1^T + YB_iK_i + \varepsilon_2 \overline{A}_i^T Y^T, \\
\Psi_{14} &= -H_3 + G_4, \\
\Psi_{15}(i) &= YC_i - Y_2, \\
\Psi_{16} &= \sqrt{d}G_1^T, \\
\Psi_{22} &= -\mathcal{S}(\varepsilon_1 Y), \\
\Psi_{23}(i) &= -G_2^T - \varepsilon_2 Y^T + \varepsilon_1 YB_iK_i, \\
\Psi_{25}(i) &= \varepsilon_1 YC_i, \\
\Psi_{26} &= \sqrt{d}G_2^T, \\
\Psi_{33}(i) &= S\left(H_2-\frac{H_1}{2}-G_3+\varepsilon_2YB_iK_i\right)-de^{-2\alpha d}Q_3, \\
\Psi_{34} &= -H_4-e^{-2\alpha d}Q_2^T-G_4, \\
\Psi_{35}(i) &= \varepsilon_2YC_i, \\
\Psi_{36} &= \sqrt{d}G_3^T, \\
\Psi_{44} &= -S\left(\frac{H_5}{2}\right)-\frac{e^{-2\alpha d}}{d}Q_1, \\
\Psi_{46} &= \sqrt{d}G_4^T, \\
\Psi_{55} &= -\mathrm{Y}_3, \\
\Psi_{66} &= -e^{-2\alpha d}U,
\end{aligned}
$$

$$
\begin{aligned}
\Phi_{11}(i) &=
\sum_{\nu=1,j}^N \lambda_{ij}^* P_j
+ 2\alpha P_i
+ \sigma\Lambda_i
+ \mathcal{S}\left(Y\overline{A}_i+YB_iK\right)
- \mathcal{Y}_1, \\
\Phi_{12}(i) &= P_i - Y + \varepsilon_1\overline{A}_i^T Y^T + (\varepsilon_1YB_iK_i)^T, \\
\Phi_{13}(i) &= YB_iK_i, \\
\Phi_{14}(i) &= YC_i - Y_2, \\
\Phi_{22} &= \mathcal{S}(-\varepsilon_1Y), \\
\Phi_{23}(i) &= \varepsilon_1YB_iK_i, \\
\Phi_{24}(i) &= \varepsilon_1YC_i, \\
\Phi_{33}(i) &= -\Lambda_i, \\
\Phi_{44} &= \delta I - \mathrm{Y}_3,
\end{aligned}
$$

$$
\begin{aligned}
\Theta_{11}(i) &= \Psi_{11}(i) + \alpha dS(H_1) + dS(H_3) + dQ_1 - Y_1, \\
\Theta_{12}(i) &= \Psi_{12}(i) + d\frac{\mathcal{S}(H_1)}{2}, \\
\Theta_{13}(i) &= \Psi_{13}(i) + 2\alpha d(H_2 - H_1) + dH_4^T + dQ_2.
\end{aligned}
$$

## Proof

The proof section is currently only a placeholder in the source notes and has not been expanded yet.
