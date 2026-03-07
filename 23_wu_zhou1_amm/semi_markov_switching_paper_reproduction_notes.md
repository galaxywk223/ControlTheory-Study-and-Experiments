# Semi-Markov Switching Paper Reproduction Notes

## Paper Information

- Title: `Event-triggered extended dissipativity stabilization of semi-Markov switching systems`
- Author note: 王凯
- Date: 2025-09-16

## Core Idea

The paper studies an event-triggered control method for semi-Markov switching systems.
The goal is to guarantee stochastic stability together with the extended dissipativity
performance index under uncertainty and external disturbances.

## Abstract Summary

- Uncertainty and external disturbances must be considered in practical systems.
- The paper proposes a new event-triggered rule that depends on the current state, the active mode, and the disturbance magnitude, which reduces unnecessary triggering more effectively than several existing methods.
- A specially constructed piecewise Lyapunov functional is used to prove stability and extended dissipativity.
- Controller gains and event-trigger parameters are obtained simultaneously by solving linear matrix inequalities (LMIs).
- Two numerical examples, a DC motor and a robotic arm, are used to validate the proposed method.

## Preliminaries

### Mathematical Model

The uncertain semi-Markov switching system is described by

$$
\begin{aligned}
\dot{x}\left( t \right)
&=
\bar{A}\left( \zeta \left( t \right) \right) x\left( t \right)
+ B\left( \zeta \left( t \right) \right) u\left( t \right)
+ C\left( \zeta \left( t \right) \right) \omega \left( t \right), \\
y\left( t \right)
&=
D\left( \zeta \left( t \right) \right) x\left( t \right), \\
x\left( 0 \right)
&=
0, \qquad
\zeta \left( 0 \right) = \zeta _0 .
\end{aligned}
$$

Here:

- $x(t)$ is the state vector.
- $u(t)$ is the control input.
- $y(t)$ is the measured output.
- $\omega(t)$ is the external disturbance.
- $\zeta(t)$ is the semi-Markov process that determines the current mode.
- $\bar{A}, B, C, D$ are mode-dependent system matrices, with $\bar{A} = A + \Delta A$.

### Three Basic Assumptions

#### Assumption 1: Controllability

The system is controllable.

This means the control input $u(t)$ can effectively influence all state variables. Without controllability, controller design is meaningless.

#### Assumption 2: Measurable Disturbance

The external disturbance is measurable.

This is a relatively strong assumption. It means the controller has access not only to the system state, but also to the disturbance magnitude in real time. This is important for building a disturbance-dependent trigger rule.

#### Assumption 3: Norm-Bounded Uncertainty

The parameter uncertainty is norm-bounded.

This means the exact uncertainty $\Delta A$ is unknown, but its magnitude is bounded in a known range, which allows robust controller design.

### Controller Form

The controller is given by

$$
u(t) = K(\zeta(t))x(t_s), \quad t \in [t_s, t_{s+1}) .
$$

Interpretation:

- $K(\zeta(t))$ is the mode-dependent controller gain matrix.
- The controller uses the last transmitted state $x(t_s)$ rather than the real-time state $x(t)$.
- On the interval $[t_s, t_{s+1})$, the control input is computed from the previously transmitted information and held until the next trigger instant.

### Event-Triggered Mechanism

The trigger condition is

$$
t_{s+1}
=
\min\left\{
t \ge t_s + d
\mid
\left( x\left( t \right) - x\left( t_s \right) \right)^T
\Lambda \left( \zeta \left( t \right) \right)
\left( x\left( t \right) - x\left( t_s \right) \right)
\ge
\sigma x^T\left( t \right)\Lambda \left( \zeta \left( t \right) \right)x\left( t \right)
+ \delta \omega ^T\left( t \right)\omega \left( t \right)
\right\}.
$$

Interpretation:

- The weighted state error energy is compared against a dynamic threshold.
- The threshold depends on the current state magnitude and the disturbance magnitude.
- The parameter $d$ enforces a minimum inter-event time and helps exclude Zeno behavior.

## Control Objectives

### Objective 1: Stochastic Stability

Definition:
the system is stochastically stable if, when $\omega(t)=0$, the expected accumulated state energy remains finite:

$$
\mathcal{E}\left\{\int_{0}^{\infty} ||x(\mu)||^2 d\mu\right\} \le M_{\circ}.
$$

This means that even though the mode changes randomly, the controller must still guarantee that the state trajectory does not diverge on average.

### Objective 2: Extended Dissipativity

The extended dissipativity condition is

$$
\sup_{t \ge 0}\{\mathcal{E}(x^T(t)\Upsilon_4 x(t))\}
\le
\mathcal{E}\left\{\int_0^t J(\mu)d\mu\right\},
$$

where

$$
J(t)=x^T(t)\Upsilon_1x(t)+2x^T(t)\Upsilon_2\omega(t)+\omega^T(t)\Upsilon_3\omega(t).
$$

This framework includes several standard performance indices as special cases, such as
$H_\infty$, $L_2-L_\infty$, and passivity.

## Main Results

### Theorem 1

For given real matrices
$\Upsilon_1 \le 0$, $\Upsilon_2$, $\Upsilon_3 > 0$, $\Upsilon_4 \ge 0$
satisfying

$$
(||\Upsilon_1||+||\Upsilon_2||) \cdot ||\Upsilon_4||=0,
$$

the paper establishes an LMI-based sufficient condition for stochastic stability and extended dissipativity of the closed-loop semi-Markov switching system.

### Theorem 2

Based on the analysis condition in Theorem 1, the paper further gives a co-design method for computing the controller gains and event-trigger matrices through equivalent LMI transformations.
