# 05 非线性时滞神经网络稳定性实验

本目录复现 [`05_非线性时滞神经网络稳定性`](../../../notes/05_非线性时滞神经网络稳定性.md) 中的稳定性判定与时域仿真结果，重点看时滞对收敛速度的影响。

## 关联笔记

- [05_非线性时滞神经网络稳定性](../../../notes/05_非线性时滞神经网络稳定性.md)

## 实验内容

- 输出三条判据对应的 LMI 裕度图。
- 比较不同时滞下的状态轨迹。
- 生成时滞与收敛时间关系图以及指定时滞下的相平面轨迹。

## 代表结果

不同延迟下的状态轨迹能够直观看到时滞增大后系统收敛速度变慢。

<p align="center">
  <img src="../../../figures/05_delay_neural_network_stability/state_trajectories_by_delay.png" alt="不同时滞下的神经网络状态轨迹" width="920" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。下面的命令都在仓库根目录执行：

```bash
python experiments/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.py
matlab -batch "run('experiments/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.m')"
```

## 输出目录

- 图像：`figures/05_delay_neural_network_stability/`
- 数值结果：`generated/05_delay_neural_network_stability/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版时滞稳定性结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
