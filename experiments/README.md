# 实验索引

`experiments/` 收集了与主笔记对应的可复现实验入口。所有脚本默认在仓库根目录运行，Python 和 MATLAB 版本尽量指向同一组图像与数值结果。

## 当前实验

| 分组 | 目录 | 对应笔记 | 主入口 | 说明 |
| --- | --- | --- | --- | --- |
| `foundations` | [01_lti_stability](./foundations/01_lti_stability/README.md) | [01 稳定性](../notes/01_线性时不变系统稳定性.md) | `python experiments/foundations/01_lti_stability/generate_results.py` | 特征值判据与 Lyapunov 判据的二维验证 |
| `foundations` | [02_lti_control](./foundations/02_lti_control/README.md) | [02 控制](../notes/02_线性时不变系统控制.md) | `python experiments/foundations/02_lti_control/generate_results.py` | 开环、状态反馈和输出反馈响应 |
| `foundations` | [03_periodic_sampling_control](./foundations/03_periodic_sampling_control/README.md) | [03 周期采样控制](../notes/03_线性时不变系统周期采样控制与稳定性分析.md) | `python experiments/foundations/03_periodic_sampling_control/generate_results.py` | 采样状态与零阶保持控制输入 |
| `robust_control` | [04_robust_control](./robust_control/04_robust_control/README.md) | [04 鲁棒控制](../notes/04_鲁棒控制.md) | `python experiments/robust_control/04_robust_control/generate_results.py` | 区间不确定系统与频域性能估计 |
| `nonlinear_and_delay` | [05_delay_neural_network_stability](./nonlinear_and_delay/05_delay_neural_network_stability/README.md) | [05 时滞神经网络稳定性](../notes/05_非线性时滞神经网络稳定性.md) | `python experiments/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.py` | 不同时滞下的稳定性和收敛时间 |
| `nonlinear_and_delay` | [06_chaotic_sync_and_image_encryption](./nonlinear_and_delay/06_chaotic_sync_and_image_encryption/README.md) | [06 同步与图像加密](../notes/06_混沌时滞神经网络同步与图像加密.md) | `python experiments/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/generate_results.py` | 混沌同步、图像加密和统计分析 |

## 运行前准备

Python 依赖见 [requirements.txt](../requirements.txt)。常用入口如下：

```bash
pip install -r requirements.txt
python experiments/foundations/01_lti_stability/generate_results.py
python experiments/robust_control/04_robust_control/generate_results.py
matlab -batch "run('experiments/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.m')"
```

运行完成后，图像会写入 `figures/`，数值结果会写入 `generated/`。
