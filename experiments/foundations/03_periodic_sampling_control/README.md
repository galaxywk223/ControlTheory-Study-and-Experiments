# 03 线性时不变系统周期采样控制实验

本目录复现 [`03_线性时不变系统周期采样控制与稳定性分析`](../../../notes/03_线性时不变系统周期采样控制与稳定性分析.md) 中的采样控制结果，重点展示采样状态、连续状态和零阶保持输入。

## 关联笔记

- [03_线性时不变系统周期采样控制与稳定性分析](../../../notes/03_线性时不变系统周期采样控制与稳定性分析.md)

## 实验内容

- 生成连续状态与采样状态对比图。
- 生成零阶保持控制输入图。
- 输出采样闭环的数值报告。

## 代表结果

连续状态和采样状态图把周期采样闭环的时域行为放在同一张图里对照展示。

<p align="center">
  <img src="../../../figures/03_periodic_sampling_control/continuous_sampled_states.png" alt="周期采样控制中的连续状态与采样状态" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。下面的命令都在仓库根目录执行：

```bash
python experiments/foundations/03_periodic_sampling_control/generate_results.py
matlab -batch "run('experiments/foundations/03_periodic_sampling_control/generate_results.m')"
```

## 输出目录

- 图像：`figures/03_periodic_sampling_control/`
- 数值结果：`generated/03_periodic_sampling_control/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版采样控制结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
