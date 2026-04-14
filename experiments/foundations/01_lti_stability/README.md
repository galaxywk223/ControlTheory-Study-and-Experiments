# 01 线性时不变系统稳定性实验

本目录复现 [`01_线性时不变系统稳定性`](../../../notes/01_线性时不变系统稳定性.md) 中的二维连续时间 LTI 系统稳定性结果，分别给出 Python 和 MATLAB 版本的数值脚本。

## 关联笔记

- [01_线性时不变系统稳定性](../../../notes/01_线性时不变系统稳定性.md)

## 实验内容

- 计算系统矩阵特征值和 Lyapunov 方程解。
- 生成状态轨迹图和相平面轨迹图。
- 把和正文结论对应的数值结果写入本地 `generated/`。

## 代表结果

状态轨迹可以直接看到二维系统从不同初值收敛到原点。

<p align="center">
  <img src="../../../figures/01_lti_stability/state_trajectories.png" alt="线性时不变系统状态轨迹" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。下面的命令都在仓库根目录执行：

```bash
python experiments/foundations/01_lti_stability/generate_results.py
matlab -batch "run('experiments/foundations/01_lti_stability/generate_results.m')"
```

## 输出目录

- 图像：`figures/01_lti_stability/`
- 数值结果：`generated/01_lti_stability/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
