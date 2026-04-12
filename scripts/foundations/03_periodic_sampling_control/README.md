# 03 线性时不变系统周期采样控制实验

本目录用于复现 [`03_线性时不变系统周期采样控制与稳定性分析`](../../../notes/03_线性时不变系统周期采样控制与稳定性分析.md) 中的采样控制结果。

## 生成内容

- 连续状态与采样状态对比图
- 零阶保持控制输入图
- 采样闭环数值报告

## 运行方式

在仓库根目录执行：

```powershell
python scripts/foundations/03_periodic_sampling_control/generate_results.py
```

```powershell
matlab -batch "run('scripts/foundations/03_periodic_sampling_control/generate_results.m')"
```

## 输出位置

- 图像：`figures/03_periodic_sampling_control/`
- 数值结果：`generated/03_periodic_sampling_control/`

## 依赖

- Python：`numpy`、`scipy`、`matplotlib`
- MATLAB：基础数值与绘图功能
