# 05 观测器与分离原理实验

本目录复现 [`05_观测器与分离原理`](../../../notes/05_观测器与分离原理.md) 中的状态估计与观测器闭环结果，重点展示估计误差衰减和分离设计后的闭环表现。

## 关联笔记

- [05_观测器与分离原理](../../../notes/05_观测器与分离原理.md)

## 实验内容

- 设计状态反馈增益和观测器增益。
- 比较真实状态与估计状态的收敛过程。
- 生成观测器闭环状态与控制输入结果并导出数值报告。

## 代表结果

估计状态图直接展示观测器误差的衰减过程。

<p align="center">
  <img src="../../../figures/05_observer_and_separation/observer_state_estimates.png" alt="观测器状态估计结果" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。以下命令在仓库根目录执行。

```bash
python experiments/foundations/05_observer_and_separation/generate_results.py
matlab -batch "run('experiments/foundations/05_observer_and_separation/generate_results.m')"
```

## 输出目录

- 图像：`figures/05_observer_and_separation/`
- 数值结果：`generated/05_observer_and_separation/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版观测器与分离原理结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
