# 01 线性时不变系统稳定性实验

本目录用于复现 [`01_线性时不变系统稳定性`](../../../notes/01_线性时不变系统稳定性.md) 中的数值结果。

## 生成内容

- 系统矩阵特征值与李雅普诺夫方程求解结果
- 状态轨迹图
- 相平面轨迹图

## 运行方式

在仓库根目录执行：

```powershell
python scripts/foundations/01_lti_stability/generate_results.py
```

```powershell
matlab -batch "run('scripts/foundations/01_lti_stability/generate_results.m')"
```

## 输出位置

- 图像：`figures/01_lti_stability/`
- 数值结果：`generated/01_lti_stability/`

## 依赖

- Python：`numpy`、`scipy`、`matplotlib`
- MATLAB：基础数值与绘图功能
