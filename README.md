# 控制理论学习与实验

这个仓库整理了现代控制理论方向的中文学习笔记和配套数值实验。主阅读层放在 `notes/`，可复现实验统一放在 `experiments/`，同一主题下尽量同时保留 Python 和 MATLAB 版本，便于把理论推导和结果图像连起来看。

## 仓库导航

- [notes/README.md](notes/README.md)：章节顺序、主题概览和阅读建议。
- [experiments/README.md](experiments/README.md)：实验索引、运行入口和目录速查。
- [figures/](figures/)：主笔记和实验 README 直接引用的图像结果。

## 学习主线

| 章节 | 主题 | 主要内容 | 实验入口 |
| --- | --- | --- | --- |
| [01](notes/01_线性时不变系统稳定性.md) | 线性时不变系统稳定性 | 用特征值判据和 Lyapunov 判据理解连续时间 LTI 系统的稳定性 | [01 稳定性实验](experiments/foundations/01_lti_stability/README.md) |
| [02](notes/02_线性时不变系统控制.md) | 线性时不变系统控制 | 把状态反馈、输出反馈和 LMI 设计条件落到可计算的闭环响应上 | [02 控制实验](experiments/foundations/02_lti_control/README.md) |
| [03](notes/03_线性时不变系统周期采样控制与稳定性分析.md) | 周期采样控制 | 观察采样、零阶保持和连续状态响应之间的关系 | [03 采样控制实验](experiments/foundations/03_periodic_sampling_control/README.md) |
| [04](notes/04_鲁棒控制.md) | 鲁棒控制 | 用区间不确定系统比较名义模型和不确定模型下的闭环表现 | [04 鲁棒控制实验](experiments/robust_control/04_robust_control/README.md) |
| [05](notes/05_非线性时滞神经网络稳定性.md) | 非线性时滞神经网络稳定性 | 通过 LMI 判据和时域仿真分析时滞对收敛速度的影响 | [05 时滞神经网络实验](experiments/nonlinear_and_delay/05_delay_neural_network_stability/README.md) |
| [06](notes/06_混沌时滞神经网络同步与图像加密.md) | 混沌同步与图像加密 | 把同步控制结果继续延伸到图像加密与解密流程 | [06 同步与加密实验](experiments/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/README.md) |

## 结果速览

| 主线 | 代表结果 | 主要看点 |
| --- | --- | --- |
| LTI 稳定性 | 多组初值的状态轨迹都收敛到原点 | 把谱判据和 Lyapunov 判据落到同一个二维系统上 |
| LTI 控制 | 状态反馈和输出反馈都能把开环发散系统拉回稳定闭环 | 对比不同控制器结构下的时域响应 |
| 周期采样控制 | 连续状态与采样状态保持一致的收敛趋势 | 直观看到采样保持和闭环更新如何一起工作 |
| 鲁棒控制 | 区间顶点的最坏频域增益估计约为 `0.47` | 不只看名义模型，还看参数漂移下的稳定裕度 |
| 时滞神经网络 | 时滞增大时，状态收敛明显变慢 | LMI 判据和时域仿真能够互相印证 |
| 混沌同步与图像加密 | 同步误差快速压低并支撑后续图像加密流程 | 从控制同步自然过渡到图像处理实验 |

## 精选展示

### 线性时不变系统稳定性

二维状态轨迹直接给出了渐近稳定最直观的时域画面。

<p align="center">
  <img src="./figures/01_lti_stability/state_trajectories.png" alt="线性时不变系统状态轨迹" width="760" />
</p>

### 鲁棒控制

参数区间扫描把名义闭环之外的稳定裕度展示得更清楚。

<p align="center">
  <img src="./figures/04_robust_control/spectral_abscissa_scan.png" alt="鲁棒控制中的谱横坐标扫描结果" width="920" />
</p>

### 混沌同步与图像加密

同步控制和图像加密被放在同一条实验链路里验证。

<p align="center">
  <img src="./figures/06_chaotic_sync_and_image_encryption/encryption_pipeline.png" alt="混沌同步与图像加密流程" width="920" />
</p>

## 快速开始

Python 依赖见 [requirements.txt](requirements.txt)。下面的命令都在仓库根目录执行：

```bash
pip install -r requirements.txt
python experiments/foundations/01_lti_stability/generate_results.py
python experiments/robust_control/04_robust_control/generate_results.py
matlab -batch "run('experiments/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/generate_results.m')"
```

图像默认写入 `figures/`，数值结果默认写入 `generated/`；`generated/` 只用于本地复现检查，不纳入版本控制。

## 仓库结构

```text
ControlTheory-Study-and-Experiments/
├─ notes/
│  ├─ README.md
│  ├─ 01_线性时不变系统稳定性.md
│  ├─ 02_线性时不变系统控制.md
│  ├─ 03_线性时不变系统周期采样控制与稳定性分析.md
│  ├─ 04_鲁棒控制.md
│  ├─ 05_非线性时滞神经网络稳定性.md
│  └─ 06_混沌时滞神经网络同步与图像加密.md
├─ experiments/
│  ├─ README.md
│  ├─ foundations/
│  ├─ robust_control/
│  └─ nonlinear_and_delay/
├─ figures/
├─ generated/
├─ requirements.txt
├─ README.md
└─ LICENSE
```

## 开源协议

本仓库中的代码、笔记、图示与文档结构基于 [MIT License](LICENSE) 开源。数据、论文内容和第三方原始资料仍以各自原始许可和引用要求为准。
