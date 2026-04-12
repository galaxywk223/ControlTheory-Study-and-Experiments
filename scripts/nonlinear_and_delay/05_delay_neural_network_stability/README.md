# 05 非线性时滞神经网络稳定性实验

本目录用于复现 [`05_非线性时滞神经网络稳定性`](../../../notes/05_非线性时滞神经网络稳定性.md) 中的稳定性判定与时域仿真结果。

## 生成内容

- 三条判据对应的 LMI 裕度图
- 不同时滞下的状态轨迹
- 时滞与收敛时间关系图
- 指定时滞下的相平面轨迹

## 运行方式

在仓库根目录执行：

```powershell
python scripts/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.py
```

```powershell
matlab -batch "run('scripts/nonlinear_and_delay/05_delay_neural_network_stability/generate_results.m')"
```

## 输出位置

- 图像：`figures/05_delay_neural_network_stability/`
- 数值结果：`generated/05_delay_neural_network_stability/`

## 依赖

- Python：`numpy`、`matplotlib`
- MATLAB：基础数值与绘图功能
