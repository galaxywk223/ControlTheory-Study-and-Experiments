# 02 线性时不变系统控制实验

本目录用于复现 [`02_线性时不变系统控制`](../../../notes/02_线性时不变系统控制.md) 中的开环与闭环响应。

## 生成内容

- 开环状态响应
- 两组状态反馈控制结果
- 三组输出反馈控制结果
- 配套数值报告

## 运行方式

在仓库根目录执行：

```powershell
python scripts/foundations/02_lti_control/generate_results.py
```

```powershell
matlab -batch "run('scripts/foundations/02_lti_control/generate_results.m')"
```

## 输出位置

- 图像：`figures/02_lti_control/`
- 数值结果：`generated/02_lti_control/`

## 依赖

- Python：`numpy`、`scipy`、`matplotlib`
- MATLAB：基础数值与绘图功能
