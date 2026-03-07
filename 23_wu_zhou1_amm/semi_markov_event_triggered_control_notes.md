# 半马尔科夫跳变系统事件触发控制笔记

整理日期：2025-09-23

## 1. 系统方程

$$
\dot {x} (t) = \bar {A} _ {\zeta (t)} x (t) + B _ {\zeta (t)} u (t) + C _ {\zeta (t)} \omega (t) \tag {1}
$$

$$
y (t) = D _ {\zeta (t)} x (t)
$$

其中， $\begin{array} { l } { { x ( 0 ) = 0 } } \\ { { \zeta ( 0 ) = \zeta _ { 0 } } } \end{array} \mathrm { { ^ o } }$ 

$$
\bar {A} _ {\zeta (t)} = A _ {\zeta (t)} + \Delta A _ {\zeta (t)} \tag {2}
$$

其中， $\varDelta A _ { \zeta ( t ) } = E _ { \zeta ( t ) } \varDelta ( t ) F _ { \zeta ( t ) }$ ，且 $\Delta ^ { T } ( t ) \varDelta ( t ) \leqslant I$ 。

## 2. 扰动不可测

构建一个 Luenberger 形式的扰动观测器：

$$
\hat {\omega} (t) = \eta (t) + L _ {\zeta (t)} x (t) \tag {3}
$$

其中，

$$
\dot {\eta} (t) = - L _ {\zeta (t)} C _ {\zeta (t)} \eta (t) - L _ {\zeta (t)} \left(C _ {\zeta (t)} L _ {\zeta (t)} + \bar {A} _ {\zeta (t)}\right) x (t) - L _ {\zeta (t)} B _ {\zeta (t)} u (t) \tag {4}
$$

定义扰动估计误差：

$$
e _ {\omega} (t) = \omega (t) - \hat {\omega} (t) \tag {5}
$$

假设扰动的变化缓慢（ $\dot { \boldsymbol { \omega } } ( t ) \approx 0 \dot { }$ ），求导得：

$$
\begin{array}{l} \dot {e} _ {\omega} (t) = \dot {\omega} (t) - \dot {\hat {\omega}} (t) \\ = 0 - \left(\dot {\eta} (t) + L _ {\zeta (t)} \dot {x} (t)\right) \\ = L _ {\zeta (t)} C _ {\zeta (t)} \left(\eta (t) + L _ {\zeta (t)} x (t) - \omega (t)\right) \tag {6} \\ = L _ {\zeta (t)} C _ {\zeta (t)} \left(\hat {\omega} (t) - \omega (t)\right) \\ = - L _ {\zeta (t)} C _ {\zeta (t)} e _ {\omega} (t) \\ \end{array}
$$

所以，原系统(1)改写为：

$$
\dot {x} (t) = \bar {A} _ {\zeta (t)} x (t) + B _ {\zeta (t)} u (t) + C _ {\zeta (t)} \left(e _ {\omega} (t) + \hat {\omega} (t)\right) \tag {7}
$$

## 3. 控制增益

$$
u (t) = K _ {\zeta (t)} x \left(t _ {s}\right) - G _ {\zeta (t)} \hat {\omega} (t), \quad t \in \left[ t _ {s}, t _ {s + 1}\right) \tag {8}
$$

## 4. 事件触发

规定事件触发的时间间隔必须大于 $d$ ，因此将式 (8) 改写为：

$$
u (t) = \left\{ \begin{array}{l l} K _ {\zeta (t)} x (t - \theta (t)) - G _ {\zeta (t)} \hat {\omega} (t) & t \in [ t _ {s}, t _ {s} + d) \\ K _ {\zeta (t)} [ x (t) + e (t) ] - G _ {\zeta (t)} \hat {\omega} (t) & t \in [ t _ {s} + d, t _ {s + 1}) \end{array} \right. \tag {9}
$$

其中，

$$
\theta (t) = t - t _ {s} \leqslant d, e (t) = x \left(t _ {s}\right) - x (t) \tag {10}
$$

我们把(7)式也改写为：

$$
\dot {x} (t) = \left\{ \begin{array}{l l} \bar {A} _ {\zeta (t)} x (t) + C _ {\zeta (t)} \left(e _ {\omega} (t) + \hat {\omega} (t)\right) & t \in [ t _ {s}, t _ {s} + d) \\ + B _ {\zeta (t)} \left(K _ {\zeta (t)} x (t - \theta (t)) - G _ {\zeta (t)} \hat {\omega} (t)\right) & \\ \bar {A} _ {\zeta (t)} x (t) + C _ {\zeta (t)} \left(e _ {\omega} (t) + \hat {\omega} (t)\right) & \\ + B _ {\zeta (t)} \left[ K _ {\zeta (t)} [ x (t) + e (t) ] - G _ {\zeta (t)} \hat {\omega} (t) \right] & t \in [ t _ {s} + d, t _ {s + 1}) \end{array} \right. \tag {11}
$$

事件触发条件：

$$
t _ {s + 1} = \min  \left\{t \geqslant t _ {s} + d \mid (x (t) - x (t _ {s})) ^ {T} \Lambda_ {\zeta (t)} (x (t) - x (t _ {s})) \geqslant \sigma x ^ {T} (t) \Lambda_ {\zeta (t)} x (t) + \delta \hat {\omega} ^ {T} (t) \hat {\omega} (t) \right\}
$$

## 5. 马尔科夫跳变概率

$$
P r \left\{\zeta (t + \iota) = j \mid \zeta (t) = i \right\} = \left\{ \begin{array}{l l} \pi_ {i j} \iota + o (\iota) & i \neq j \\ 1 + \pi_ {i i} \iota + o (\iota) & i = j \end{array} \right. \tag {12}
$$

概率和为1：

$$
\pi_ {i i} (\iota) = - \sum_ {j \neq i} \pi_ {i j} (\iota) \tag {13}
$$

令 $\zeta ( t ) = i$ ，我们替换后得到：

$$
\begin{array}{l} \dot {x} (t) = \bar {A} _ {i} x (t) + B _ {i} u (t) + C _ {i} \left(e _ {\omega} (t) + \hat {\omega} (t)\right) \\ \dot {e} _ {\omega} (t) = - L _ {i} C _ {i} e _ {\omega} (t) \\ y (t) = D _ {i} x (t) \tag {14} \\ u (t) = K _ {i} x \left(t _ {s}\right) - G _ {i} \hat {\omega} (t), t \in \left[ t _ {s}, t _ {s + 1}\right) \\ \hat {\omega} (t) = \eta (t) + L _ {i} x (t) \\ \end{array}
$$

其中，$x ( 0 ) = 0$，$\zeta ( 0 ) = \zeta _ { 0 }$。

$$
\bar {A} _ {i} = A _ {i} + \Delta A _ {i}
$$

$$
\Delta A _ {i} = E _ {i} \Delta (t) F _ {i}, \Delta^ {T} (t) \Delta (t) \leqslant I
$$

$$
t _ {s + 1} = \min  \left\{t \geqslant t _ {s} + d \mid \left(x (t) - x \left(t _ {s}\right)\right) ^ {T} \Lambda_ {i} \left(x (t) - x \left(t _ {s}\right)\right) \geqslant \sigma x ^ {T} (t) \Lambda_ {i} x (t) + \delta \hat {\omega} ^ {T} (t) \hat {\omega} (t) \right\}
$$

$$
\dot {\eta} (t) = - L _ {i} C _ {i} \eta (t) - L _ {i} \left(C _ {i} L _ {i} + \bar {A} _ {i}\right) x (t) - L _ {i} B _ {i} u (t)
$$

## 6. 增广系统

定义增广状态

$$
\tilde { x } ( t ) = \biggl [ \begin{array} { l } { x ( t ) } \\ { e _ { \omega } ( t ) } \end{array} \biggr ] 。
$$

### 6.1 情况 1

适用区间：$t \in [ t _ { s } , t _ { s } + d )$。

$$
u (t) = K _ {i} x (t - \theta (t)) - G _ {i} \hat {\omega} (t) \tag {15}
$$

代入得：

$$
\dot {\tilde {x}} (t) = \left[ \begin{array}{c} \bar {A} _ {i} x (t) + B _ {i} K _ {i} x (t - \theta (t)) + \left(C _ {i} - B _ {i} G _ {i}\right) \hat {\omega} (t) + C _ {i} e _ {\omega} (t) \\ - L _ {i} C _ {i} e _ {\omega} (t) \end{array} \right] \tag {16}
$$

其中， $\theta ( t ) = t - t _ { s } \leqslant d$ 。

### 6.2 情况 2

适用区间：$t \in [ t _ { s } + d , t _ { s + 1 } )$。

$$
u (t) = K _ {i} (x (t) + e (t)) - G _ {i} \hat {\omega} (t) \tag {17}
$$

代入得：

$$
\dot {\hat {x}} (t) = \left[ \begin{array}{c} \left(\bar {A} _ {i} + B _ {i} K _ {i}\right) x (t) + B _ {i} K _ {i} e (t) + \left(C _ {i} - B _ {i} G _ {i}\right) \hat {\omega} (t) + C _ {i} e _ {\omega} (t) \\ - L _ {i} C _ {i} e _ {\omega} (t) \end{array} \right] \tag {18}
$$

其中， $e ( t ) = x ( t _ { s } ) - x ( t )$ 。

## 7. 构建新的 Lyapunov-Krasovskii 泛函 (LKF)

定义 LKF 为

$$
V (t, i) = \left\{ \begin{array}{l l} V _ {\alpha} (t, i), & t \in [ t _ {s}, t _ {s} + d) \\ V _ {\beta} (t, i), & t \in [ t _ {s} + d, t _ {s + 1}) \end{array} \right. \tag {19}
$$

其中，

$$
V _ {\alpha} (t, i) = \sum_ {m = 1} ^ {4} V _ {m} (t, i) \tag {20}
$$

$$
V _ {\beta} (t, i) = V _ {1} (t, i)
$$

且：

$$
V _ {1} (t, i) = e ^ {2 \alpha t} \tilde {x} ^ {T} (t) \tilde {P} _ {i} \tilde {x} (t) = e ^ {2 \alpha t} \left[ \begin{array}{c} x (t) \\ e _ {\omega} (t) \end{array} \right] ^ {T} \left[ \begin{array}{c c} P _ {1 1, i} & P _ {1 2, i} \\ \star & P _ {2 2, i} \end{array} \right] \left[ \begin{array}{c} x (t) \\ e _ {\omega} (t) \end{array} \right]
$$

$$
V _ {2} (t, i) = \theta_ {d} (t) \int_ {t - \theta (t)} ^ {t} e ^ {2 a \mu} \dot {x} ^ {T} (\mu) U \dot {x} (\mu) d \mu \tag {21}
$$

$$
V _ {3} (t, i) = \theta_ {d} (t) \int_ {t - \theta (t)} ^ {t} e ^ {2 a \mu} \eta_ {1} ^ {T} (\mu) \bar {Q} \eta_ {1} (\mu) d \mu
$$

$$
V _ {4} (t, i) = \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {T} (t) F \eta_ {2} (t)
$$

其中，

$$
\begin{array}{l} \theta_ {d} (t) = d - \theta (t), \eta_ {1} (\mu) = c o l \left\{x (\mu), x (\mu - \theta (\mu)) \right\} \\ \eta_ {2} (t) = \operatorname {c o l} \left\{x (t), x (t - \theta (t)), \int_ {t - \theta (t)} ^ {t} x (\mu) \mathrm {d} \mu \right\} \\ \mathrm {F} = \left[ \begin{array}{c c c} \mathcal {S} \left(\frac {H _ {1}}{2}\right) & - H _ {1} + H _ {2} & H _ {3} \\ * & \mathcal {S} \left(- H _ {2} + \frac {H _ {1}}{2}\right) & H _ {4} \\ * & * & \mathcal {S} \left(\frac {H _ {5}}{2}\right) \end{array} \right] \tag {22} \\ \end{array}
$$

且

$$
\bar {Q} = \left[ \begin{array}{l l} Q _ {1} & Q _ {2} \\ * & Q _ {3} \end{array} \right] > 0 \tag {23}
$$

定义：

$$
\xi (t) = \operatorname {c o l} \left\{x (t), e _ {\omega} (t), x (t - \theta (t)), \int_ {t - \theta (t)} ^ {t} x (\mu) d \mu \right\} \tag {24}
$$

所以

$$
V _ {1} (t, i) = e ^ {2 \alpha t} \left[ \begin{array}{c} x (t) \\ e _ {\omega} (t) \end{array} \right] ^ {T} \left[ \begin{array}{c c} P _ {1 1, i} & P _ {1 2, i} \\ \star & P _ {2 2, i} \end{array} \right] \left[ \begin{array}{c} x (t) \\ e _ {\omega} (t) \end{array} \right] = e ^ {2 \alpha t} \xi^ {T} (t) \left[ \begin{array}{c c c c} P _ {1 1, i} & P _ {1 2, i} & 0 & 0 \\ * & P _ {2 2, i} & 0 & 0 \\ * & * & 0 & 0 \\ * & * & * & 0 \end{array} \right] \xi (t)
$$

$$
\begin{array}{l} V _ {4} (t, i) = \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {T} (t) F \eta_ {2} (t) \\ = \theta_ {d} (t) e ^ {2 \alpha t} \xi^ {T} (t) \left[ \begin{array}{c c c c} S \left(\frac {H _ {1}}{2}\right) & 0 & - H _ {1} + H _ {2} & H _ {3} \\ * & 0 & 0 & 0 \\ * & * & S \left(- H _ {2} + \frac {H _ {1}}{2}\right) & H _ {4} \\ * & * & * & S \left(\frac {H _ {5}}{2}\right) \end{array} \right] \xi (t) \tag {25} \\ \end{array}
$$

所以：

$$
\begin{array}{l} V _ {1} (t, i) + V _ {4} (t, i) \\ = e ^ {2 \alpha t} \xi^ {T} (t) \left[ \begin{array}{c c c c} P _ {1 1, i} + \theta_ {d} (t) \mathcal {S} \left(\frac {H _ {1}}{2}\right) & P _ {1 2, i} & \theta_ {d} (t) (- H _ {1} + H _ {2}) & \theta_ {d} (t) H _ {3} \\ * & P _ {2 2, i} & 0 & 0 \\ * & * & \theta_ {d} (t) \mathcal {S} \left(- H _ {2} + \frac {H _ {1}}{2}\right) & \theta_ {d} (t) H _ {4} \\ * & * & * & \theta_ {d} (t) \mathcal {S} \left(\frac {H _ {5}}{2}\right) \end{array} \right] \xi (t) \tag {26} \\ \end{array}
$$

后续和原论文类似，易证 $V ( t ) > 0$ 。

## 8. 最小生成元的计算

$$
\mathcal {L} V (t, i) = \frac {d}{d t} V (t, i) + \sum_ {j = 1} ^ {N} \pi_ {i j} V (t, j)
$$

### 8.1 情况 1

适用区间：$t \in [ t _ { s } , t _ { s } + d )$。

$$
\begin{array}{l} \mathcal {L} V _ {1} (t, i) = e ^ {2 \alpha t} \left[ 2 \alpha \tilde {x} ^ {T} \tilde {P} _ {i} \tilde {x} + 2 \tilde {x} ^ {T} \tilde {P} _ {i} \dot {\tilde {x}} (t) + \tilde {x} ^ {T} \left(\sum_ {j = 1} ^ {N} \pi_ {i j} \tilde {P} _ {j}\right) \tilde {x} \right] \\ = e ^ {2 \alpha t} \left[ 2 \alpha \tilde {x} ^ {T} \tilde {P} \tilde {x} + \tilde {x} ^ {T} \left(\sum_ {j = 1} ^ {N} \pi_ {i j} \tilde {P} _ {j}\right) \tilde {x} + 2 \left(x ^ {T} P _ {1 1} + e _ {\omega} ^ {T} P _ {1 2} ^ {T}\right) \left(\bar {A} x + B K x (t - \theta) + (C - B G) \hat {\omega} + C e _ {\omega}\right) - 2 \right. \\ \end{array}
$$

$$
\begin{array}{l} \mathcal {L} V _ {2} (t, i) = \theta_ {d} (t) e ^ {2 a t} \dot {x} ^ {T} (t) U \dot {x} (t) - \int_ {t - \theta (t)} ^ {t} e ^ {2 a \mu} \dot {x} ^ {T} (\mu) U \dot {x} (\mu) d \mu \\ \leqslant \theta_ {d} (t) e ^ {2 a t} \dot {x} ^ {T} (t) U \dot {x} (t) - e ^ {2 a t} e ^ {- 2 a d} \int_ {t - \theta (t)} ^ {t} \dot {x} ^ {T} (\mu) U \dot {x} (\mu) d \mu \\ \end{array}
$$

$$
\begin{array}{l} \mathcal {L} V _ {3} (t, i) = \theta_ {d} (t) e ^ {2 a t} \eta_ {1} ^ {T} (t) \bar {Q} \eta_ {1} (t) - \int_ {t - \theta (t)} ^ {t} e ^ {2 a \mu} \eta_ {1} ^ {T} (\mu) \bar {Q} \eta_ {1} (\mu) d \mu \\ \leqslant \theta_ {d} (t) e ^ {2 a t} \eta_ {1} ^ {T} (t) \bar {Q} \eta_ {1} (t) - e ^ {2 a t} e ^ {- 2 a d} \int_ {t - \theta (t)} ^ {t} \eta_ {1} ^ {T} (\mu) \bar {Q} \eta_ {1} (\mu) d \mu \\ \end{array}
$$

$$
\begin{array}{l} \mathcal {L} V _ {4} (t, i) = - e ^ {2 \alpha t} \eta_ {2} ^ {T} (\boldsymbol {t}) F \eta_ {2} (\boldsymbol {t}) + 2 \alpha \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {T} (t) F \eta_ {2} (t) + 2 \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {T} F \dot {\eta} _ {2} (t) \\ \leq - e ^ {2 \alpha t} \eta_ {2} ^ {T} (t) F \eta_ {2} (t) + 2 \alpha \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {T} (t) F \eta_ {2} (t) \\ + 2 \theta_ {d} (t) e ^ {2 \alpha t} x ^ {T} (t) \frac {H _ {1} + H _ {1} ^ {T}}{2} \dot {x} (t) + 2 \theta_ {d} (t) e ^ {2 \alpha t} x ^ {T} (t - \theta (t)) (- H _ {1} + H _ {2}) ^ {T} \dot {x} (t) \\ + 2 \theta_ {d} (t) e ^ {2 \alpha t} \int_ {t - \theta (t)} ^ {t} x ^ {T} (\mu) d \mu H _ {3} ^ {T} \dot {x} (t) + 2 \theta_ {d} (t) e ^ {2 \alpha t} x ^ {T} (t) H _ {3} x (t) \\ + 2 \theta_ {d} (t) e ^ {2 \alpha t} x ^ {T} (t - \theta (t)) H _ {4} x (t) + 2 \theta_ {d} (t) e ^ {2 \alpha t} \int_ {t - \theta (t)} ^ {t} x ^ {T} (\mu) d \mu \frac {H _ {5} + H _ {5} ^ {T}}{2} x (t) \\ \end{array}
$$

构建 0 等式：

$$
\begin{array}{l} 2 \left[ x ^ {T} + \dot {x} ^ {T} \mathcal {E} _ {1} + e _ {\omega} ^ {T} \mathcal {E} _ {2} + \hat {\omega} ^ {T} \mathcal {E} _ {3} + x ^ {T} (t - \theta) \mathcal {E} _ {4} \right] \mathcal {Y} _ {i} ^ {T} \\ \times \left[ - \dot {x} + \bar {A} _ {i} x + B _ {i} K _ {i} x (t - \theta) + \left(C _ {i} - B _ {i} G _ {i}\right) \hat {\omega} + C _ {i} e _ {\omega} \right] = 0 \tag {28} \\ \end{array}
$$

### 8.2 情况 2

适用区间：$t \in [ t _ { s } + d , t _ { s + 1 } )$。
