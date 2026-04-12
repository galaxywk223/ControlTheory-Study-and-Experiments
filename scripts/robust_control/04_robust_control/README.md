# 04 鲁棒控制实验

本目录用于复现 [`04_鲁棒控制`](../../../notes/04_鲁棒控制.md) 中的区间不确定系统实验。

## 生成内容

- 参数区间谱横坐标扫描图
- 顶点处频域增益估计图
- 多组参数下的状态响应与控制输入
- 顶点指标表和汇总结果

## 运行方式

在仓库根目录执行：

```powershell
python scripts/robust_control/04_robust_control/generate_results.py
```

```powershell
matlab -batch "run('scripts/robust_control/04_robust_control/generate_results.m')"
```

## 输出位置

- 图像：`figures/04_robust_control/`
- 数值结果：`generated/04_robust_control/`

## 依赖

- Python：`numpy`、`scipy`、`matplotlib`
- MATLAB：基础数值与绘图功能
