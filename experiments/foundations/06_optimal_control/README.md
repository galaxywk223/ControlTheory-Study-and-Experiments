# 06 最优控制实验

本目录复现 [`06_最优控制`](../../../notes/06_最优控制.md) 中的连续时间 LQR 结果，重点比较不同权重选择下的状态响应和控制能量。

## 关联笔记

- [06_最优控制](../../../notes/06_最优控制.md)

## 实验内容

- 求解连续时间 Riccati 方程并恢复 LQR 反馈增益。
- 比较两组 `(Q,R)` 权重下的闭环状态响应。
- 比较两组权重下的控制输入并导出数值报告。

## 代表结果

状态响应图用于观察不同 LQR 权重对应的收敛速度差异。

<p align="center">
  <img src="../../../figures/06_optimal_control/lqr_weight_tradeoff_states.png" alt="不同 LQR 权重下的状态响应对比" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。以下命令在仓库根目录执行。

```bash
python experiments/foundations/06_optimal_control/generate_results.py
matlab -batch "run('experiments/foundations/06_optimal_control/generate_results.m')"
```

## 输出目录

- 图像：`figures/06_optimal_control/`
- 数值结果：`generated/06_optimal_control/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版最优控制结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
