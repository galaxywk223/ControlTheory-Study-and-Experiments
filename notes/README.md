# 学习笔记

`notes/` 是仓库的主阅读层。各章节统一组织问题背景、核心结论、数值结果和实验入口。根目录 [README](../README.md) 提供总览，后续章节按下列顺序组织。

## 章节顺序

| 章节 | 主题 | 内容定位 | 实验入口 |
| --- | --- | --- | --- |
| [01_系统建模与状态空间基础](./01_系统建模与状态空间基础.md) | 建模与状态空间 | 物理系统到状态空间模型的统一入口 | [01 建模实验](../experiments/foundations/01_state_space_modeling/README.md) |
| [02_线性时不变系统稳定性](./02_线性时不变系统稳定性.md) | LTI 稳定性 | 连续时间 LTI 稳定性入门 | [02 稳定性实验](../experiments/foundations/02_lti_stability/README.md) |
| [03_可控性与可观性](./03_可控性与可观性.md) | 可控与可观 | 反馈设计与状态估计的前置条件 | [03 可控可观实验](../experiments/foundations/03_controllability_observability/README.md) |
| [04_线性时不变系统控制](./04_线性时不变系统控制.md) | 状态反馈与输出反馈 | 控制器设计条件与 LMI 综合 | [04 控制实验](../experiments/foundations/04_lti_control/README.md) |
| [05_观测器与分离原理](./05_观测器与分离原理.md) | 状态估计 | 观测器设计与分离原理 | [05 观测器实验](../experiments/foundations/05_observer_and_separation/README.md) |
| [06_最优控制](./06_最优控制.md) | LQR 与 Riccati | 稳定化与性能折中设计 | [06 最优控制实验](../experiments/foundations/06_optimal_control/README.md) |
| [07_跟踪与抗扰](./07_跟踪与抗扰.md) | 伺服控制 | 参考跟踪、积分器与扰动抑制 | [07 跟踪抗扰实验](../experiments/foundations/07_tracking_and_disturbance_rejection/README.md) |
| [08_线性时不变系统周期采样控制与稳定性分析](./08_线性时不变系统周期采样控制与稳定性分析.md) | 周期采样控制 | 采样保持与时滞结构分析 | [08 采样控制实验](../experiments/foundations/08_periodic_sampling_control/README.md) |
| [09_鲁棒控制](./09_鲁棒控制.md) | 区间不确定系统与鲁棒性能 | 不确定模型与鲁棒性能 | [09 鲁棒控制实验](../experiments/robust_control/09_robust_control/README.md) |
| [10_非线性时滞神经网络稳定性](./10_非线性时滞神经网络稳定性.md) | 非线性与时滞稳定性 | 非线性时滞稳定性分析 | [10 时滞神经网络实验](../experiments/nonlinear_and_delay/10_delay_neural_network_stability/README.md) |
| [11_混沌时滞神经网络同步与图像加密](./11_混沌时滞神经网络同步与图像加密.md) | 混沌同步与图像加密 | 主线后的专题应用 | [11 同步与加密实验](../experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/README.md) |

## 与实验对应

| 实验目录 | 对应章节 | 说明 |
| --- | --- | --- |
| [experiments/foundations/01_state_space_modeling](../experiments/foundations/01_state_space_modeling/README.md) | `01` | 质量-弹簧-阻尼模型、状态响应和平衡点 |
| [experiments/foundations/02_lti_stability](../experiments/foundations/02_lti_stability/README.md) | `02` | 特征值判据、Lyapunov 方程和二维相图 |
| [experiments/foundations/03_controllability_observability](../experiments/foundations/03_controllability_observability/README.md) | `03` | 最小能量状态转移与状态重构 |
| [experiments/foundations/04_lti_control](../experiments/foundations/04_lti_control/README.md) | `04` | 开环、状态反馈和输出反馈响应对比 |
| [experiments/foundations/05_observer_and_separation](../experiments/foundations/05_observer_and_separation/README.md) | `05` | 状态估计、误差收敛与观测器闭环 |
| [experiments/foundations/06_optimal_control](../experiments/foundations/06_optimal_control/README.md) | `06` | LQR 权重对状态与控制输入的影响 |
| [experiments/foundations/07_tracking_and_disturbance_rejection](../experiments/foundations/07_tracking_and_disturbance_rejection/README.md) | `07` | 参考跟踪、积分器与扰动抑制 |
| [experiments/foundations/08_periodic_sampling_control](../experiments/foundations/08_periodic_sampling_control/README.md) | `08` | 采样状态、连续状态和零阶保持输入 |
| [experiments/robust_control/09_robust_control](../experiments/robust_control/09_robust_control/README.md) | `09` | 区间不确定系统的鲁棒稳定与性能估计 |
| [experiments/nonlinear_and_delay/10_delay_neural_network_stability](../experiments/nonlinear_and_delay/10_delay_neural_network_stability/README.md) | `10` | LMI 裕度、相图和收敛时间扫描 |
| [experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption](../experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/README.md) | `11` | 混沌同步误差、图像加密和统计分析 |
