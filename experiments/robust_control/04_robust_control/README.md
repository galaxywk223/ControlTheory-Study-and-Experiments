# 04 鲁棒控制实验

本目录复现 [`04_鲁棒控制`](../../../notes/04_鲁棒控制.md) 中的区间不确定系统实验，重点观察参数漂移下的闭环稳定性和频域性能估计。

## 关联笔记

- [04_鲁棒控制](../../../notes/04_鲁棒控制.md)

## 实验内容

- 生成参数区间内的谱横坐标扫描图。
- 估计区间顶点处的频域增益。
- 输出多组参数下的状态响应、控制输入和汇总结果。

## 代表结果

开环与闭环谱横坐标对比表明，同一反馈增益能把扫描区域内的闭环极点整体压回左半平面。

<p align="center">
  <img src="../../../figures/04_robust_control/spectral_abscissa_scan.png" alt="鲁棒控制中的开环与闭环谱横坐标扫描结果" width="920" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。下面的命令都在仓库根目录执行：

```bash
python experiments/robust_control/04_robust_control/generate_results.py
matlab -batch "run('experiments/robust_control/04_robust_control/generate_results.m')"
```

## 输出目录

- 图像：`figures/04_robust_control/`
- 数值结果：`generated/04_robust_control/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版鲁棒控制结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
