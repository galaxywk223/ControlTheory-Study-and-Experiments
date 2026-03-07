# Markov Jump Event-Triggered Design Notes

## 1. Problem Restatement

### 1.1 Markov Jump Plant

Consider the continuous-time Markov jump linear system with mode set
$\mathcal { N } = \left\{ 1 , \ldots , N \right\}$:

$$
\begin{array}{l}
\dot {x} (t) = \left(A _ {r (t)} + \Delta A _ {r (t)}\right) x (t) + B _ {r (t)} u (t) + C _ {r (t)} w (t), \\
y (t) = D _ {r (t)} x (t), \quad x (0) = 0, \quad r (0) = r _ {0}.
\end{array}
$$

Here, $w \in \mathbb { R } ^ { k }$ is an unmeasurable external disturbance, and
$r ( t )$ is a homogeneous continuous-time Markov chain with generator
$\Pi = [ \pi _ { i j } ] _ { N \times N }$, where
$\pi _ { i j } \ge 0$ for $i \ne j$ and $\sum _ { j } \pi _ { i j } = 0$.

The uncertainty is assumed to have the norm-bounded structure

$$
\Delta A _ {i} = E _ {i} \Xi (t) F _ {i}, \quad \Xi^ {\top} (t) \Xi (t) \leq I .
$$

### 1.2 Event-Triggered Mechanism (MDSETM)

Replace the true disturbance in the trigger threshold by its implementable estimate
(see Section 1.3). The triggering instants
$0 = t _ { 0 } < t _ { 1 } < \dots < t _ { s } < \dots$
satisfy, with minimum sampling interval $d > 0$,

$$
t _ {s + 1} = \min  \left\{t \geq t _ {s} + d \mid (x (t) - x (t _ {s})) ^ {\top} \Phi_ {r (t)} (x (t) - x (t _ {s})) \geq \sigma x ^ {\top} (t) \Phi_ {r (t)} x (t) + \delta \| \hat {w} (t) \| ^ {2} \right\}.
$$

Here, $\Phi _ { i } \succeq 0$ is the mode-dependent trigger matrix, and
$\sigma \geq 0$, $\delta \geq 0$.

The control law is implemented in a two-stage form:

$$
u (t) = \left\{
\begin{array}{l l}
K _ {r (t)} x (t - \theta (t)), &
\theta (t) = t - t _ {s} \in [ 0, d ], \quad t \in [ t _ {s}, t _ {s} + d), \\
K _ {r (t)} [ x (t) + e (t) ], &
e (t) = x \left(t _ {s}\right) - x (t), \quad t \in [ t _ {s} + d, t _ {s + 1}).
\end{array}
\right.
$$

This yields a two-stage time/mode switched closed-loop structure and excludes Zeno behavior.

### 1.3 Unmeasurable Disturbance and Observer Setting

Since $w ( t )$ is not directly measurable, introduce a disturbance observer to estimate
both the state and disturbance. Assume the disturbance satisfies the slow-varying model

$$
\dot {w} (t) = S w (t) + v (t),
$$

where $v$ denotes unknown disturbance variation or noise.

For mode $i = r(t)$, consider the mode-dependent observer

$$
\begin{array}{l}
\dot {\hat {x}} = A _ {i} \hat {x} + B _ {i} u + C _ {i} \hat {w} + L _ {i} (y - D _ {i} \hat {x}), \\
\dot {\hat {w}} = \Gamma_ {i} (y - D _ {i} \hat {x}) .
\end{array}
$$

Let $e _ { x } = x - \hat { x }$ and $e _ { w } = w - \hat { w }$. Then

$$
\dot {e} = \mathcal {A} _ {i} e + \mathcal {B} _ {i} v,
\quad
e = \left[ \begin{array}{l} e _ {x} \\ e _ {w} \end{array} \right].
$$

### 1.4 Performance Index: Extended Dissipativity

Given matrices $\Upsilon _ { 1 } \leq 0$, $\Upsilon _ { 2 }$,
$\Upsilon _ { 3 } > 0$, $\Upsilon _ { 4 } \geq 0$ satisfying
$( | | \Upsilon _ { 1 } | | + | | \Upsilon _ { 2 } | | ) | | \Upsilon _ { 4 } | | = 0$,
define

$$
J (t) = x ^ {\top} \Upsilon_ {1} x + 2 x ^ {\top} \Upsilon_ {2} w + w ^ {\top} \Upsilon_ {3} w.
$$

If, under $x ( 0 ) = 0$, the inequality

$$
\sup  _ {t \geq 0} \mathbb {E} \left\{x ^ {\top} (t) \Upsilon_ {4} x (t) \right\}
\leq
\mathbb {E} \left\{\int_ {0} ^ {t} J (\mu) d \mu \right\}
$$

holds for all $t \geq 0$, then the closed-loop system is said to satisfy extended dissipativity.
Special cases include $H _ { \infty }$, $L _ { 2 } - L _ { \infty }$, and passivity.

### 1.5 Design Objective

Under the MJS + MDSETM + disturbance observer framework, co-design the mode-dependent gains
$\{ K _ { i } , \Phi _ { i } , L _ { i } , \Gamma _ { i } \}$ and free matrices such that the
closed-loop system is mean-square stable under Markov jumps and uncertainty, while also satisfying
extended dissipativity. At the same time, maintain a low number of trigger events through the
disturbance-estimate-dependent threshold.

## 2. Piecewise Lyapunov Functional and Key Inequalities

For compact notation, define

$$
\theta_ {d} (t) = d - \theta (t),
$$

$$
\eta_ {1} (\mu) = \operatorname {c o l} \{x (\mu), x (\mu - \theta (\mu)) \},
$$

$$
\eta_ {2} (t) = \operatorname {c o l} \left\{x (t), x (t - \theta (t)), \int_ {t - \theta (t)} ^ {t} x (\mu) d \mu \right\}.
$$

### 2.1 Piecewise Lyapunov Functional

Construct the time- and mode-dependent piecewise Lyapunov functional

$$
V (t, i) = \left\{
\begin{array}{l l}
V _ {\alpha} (t, i), & t \in \left[ t _ {s}, t _ {s} + d\right), \\
V _ {\beta} (t, i), & t \in \left[ t _ {s} + d, t _ {s + 1}\right),
\end{array}
\right.
\quad i = r (t).
$$

Set $V _ { \beta } = V _ { 1 }$, and define

$$
\begin{array}{l}
V _ {1} (t, i) = e ^ {2 \alpha t} x ^ {\top} (t) P _ {i} x (t), \\
V _ {2} (t, i) = \theta_ {d} (t) \int_ {t - \theta (t)} ^ {t} e ^ {2 \alpha \mu} \dot {x} ^ {\top} (\mu) U \dot {x} (\mu) d \mu , \\
V _ {3} (t, i) = \theta_ {d} (t) \int_ {t - \theta (t)} ^ {t} e ^ {2 \alpha \mu} \eta_ {1} ^ {\top} (\mu) \bar {Q} \eta_ {1} (\mu) d \mu , \\
\bar {Q} = \left[ \begin{array}{c c} Q _ {1} & Q _ {2} \\ * & Q _ {3} \end{array} \right] > 0, \\
V _ {4} (t, i) = \theta_ {d} (t) e ^ {2 \alpha t} \eta_ {2} ^ {\top} (t) F \eta_ {2} (t), \\
F = \left[ \begin{array}{c c c}
S \left(\frac {H _ {1}}{2}\right) & - H _ {1} + H _ {2} & H _ {3} \\
* & S \left(- H _ {2} + \frac {H _ {1}}{2}\right) & H _ {4} \\
* & * & S \left(\frac {H _ {5}}{2}\right)
\end{array} \right].
\end{array}
$$

By $P _ { i } > 0$, $U > 0$, $\bar {Q} > 0$, and the chosen construction, one has the lower bound

$$
V (t, i)
\geq
e ^ {2 \alpha t} \eta_ {2} ^ {\top} (t)
\left[ \begin{array}{l l l}
\Upsilon_ {4} & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{array} \right]
\eta_ {2} (t)
\geq
x ^ {\top} (t) \Upsilon_ {4} x (t).
$$

This gives the lower bound needed for the terminal-state term in the extended dissipativity condition.

### 2.2 Technical Lemmas

#### 2.2.1 Jensen Inequality

Use Jensen's inequality to upper-bound interval integral terms by endpoint or averaged quadratic
forms, so they can be embedded into LMIs.

#### 2.2.2 Norm-Bounded Uncertainty Lemma

For the uncertain term
$\Delta A _ { i } = E _ { i } \Xi F _ { i }$ with $\Xi ^ { \top } \Xi \le I$,
use a slack-variable-based bound to transform the uncertain quadratic term into a deterministic
matrix inequality.

Young's inequality is also used to bound cross terms involving the observer error and related
coupling terms, which are then absorbed into diagonal blocks of the final matrix inequality.
