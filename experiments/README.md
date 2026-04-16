# 实验索引

`experiments/` 收集了与主笔记对应的可复现实验入口。所有脚本默认在仓库根目录运行，Python 和 MATLAB 版本尽量指向同一组图像与数值结果。

## 当前实验

| 分组 | 目录 | 对应笔记 | 主入口 | 说明 |
| --- | --- | --- | --- | --- |
| `foundations` | [01_state_space_modeling](./foundations/01_state_space_modeling/README.md) | [01 建模基础](../notes/01_系统建模与状态空间基础.md) | `python experiments/foundations/01_state_space_modeling/generate_results.py` | 状态空间建模、平衡点与输入响应 |
| `foundations` | [02_lti_stability](./foundations/02_lti_stability/README.md) | [02 稳定性](../notes/02_线性时不变系统稳定性.md) | `python experiments/foundations/02_lti_stability/generate_results.py` | 特征值判据与 Lyapunov 判据的二维验证 |
| `foundations` | [03_controllability_observability](./foundations/03_controllability_observability/README.md) | [03 可控可观](../notes/03_可控性与可观性.md) | `python experiments/foundations/03_controllability_observability/generate_results.py` | 最小能量状态转移与状态重构 |
| `foundations` | [04_lti_control](./foundations/04_lti_control/README.md) | [04 控制](../notes/04_线性时不变系统控制.md) | `python experiments/foundations/04_lti_control/generate_results.py` | 开环、状态反馈和输出反馈响应 |
| `foundations` | [05_observer_and_separation](./foundations/05_observer_and_separation/README.md) | [05 观测器](../notes/05_观测器与分离原理.md) | `python experiments/foundations/05_observer_and_separation/generate_results.py` | 状态估计、误差收敛与分离设计 |
| `foundations` | [06_optimal_control](./foundations/06_optimal_control/README.md) | [06 最优控制](../notes/06_最优控制.md) | `python experiments/foundations/06_optimal_control/generate_results.py` | 连续时间 LQR 与权重折中 |
| `foundations` | [07_tracking_and_disturbance_rejection](./foundations/07_tracking_and_disturbance_rejection/README.md) | [07 跟踪抗扰](../notes/07_跟踪与抗扰.md) | `python experiments/foundations/07_tracking_and_disturbance_rejection/generate_results.py` | 参考跟踪、积分器与常值扰动抑制 |
| `foundations` | [08_periodic_sampling_control](./foundations/08_periodic_sampling_control/README.md) | [08 周期采样控制](../notes/08_线性时不变系统周期采样控制与稳定性分析.md) | `python experiments/foundations/08_periodic_sampling_control/generate_results.py` | 采样状态与零阶保持控制输入 |
| `robust_control` | [09_robust_control](./robust_control/09_robust_control/README.md) | [09 鲁棒控制](../notes/09_鲁棒控制.md) | `python experiments/robust_control/09_robust_control/generate_results.py` | 区间不确定系统与频域性能估计 |
| `nonlinear_and_delay` | [10_delay_neural_network_stability](./nonlinear_and_delay/10_delay_neural_network_stability/README.md) | [10 时滞神经网络稳定性](../notes/10_非线性时滞神经网络稳定性.md) | `python experiments/nonlinear_and_delay/10_delay_neural_network_stability/generate_results.py` | 不同时滞下的稳定性和收敛时间 |
| `nonlinear_and_delay` | [11_chaotic_sync_and_image_encryption](./nonlinear_and_delay/11_chaotic_sync_and_image_encryption/README.md) | [11 同步与图像加密](../notes/11_混沌时滞神经网络同步与图像加密.md) | `python experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/generate_results.py` | 混沌同步、图像加密和统计分析 |

## 运行前准备

Python 依赖见 [requirements.txt](../requirements.txt)。常用入口如下：

```bash
pip install -r requirements.txt
python experiments/foundations/01_state_space_modeling/generate_results.py
python experiments/foundations/06_optimal_control/generate_results.py
matlab -batch "run('experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/generate_results.m')"
```

运行完成后，图像会写入 `figures/`，数值结果会写入 `generated/`。
