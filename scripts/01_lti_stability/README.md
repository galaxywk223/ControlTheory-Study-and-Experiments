# 01 线性时不变系统稳定性脚本

本目录用于复现 [线性时不变系统稳定性笔记](../../01_线性时不变系统稳定性.md) 中的数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

两份脚本均完成以下内容：

- 计算系统矩阵特征值
- 求解李雅普诺夫方程 `A^T P + P A = -I`
- 绘制状态轨迹
- 绘制相平面轨迹

## 输出位置

- 图像：`figures/01_lti_stability/`
- 数值结果：`generated/01_lti_stability/`

## 运行方式

```powershell
python scripts/01_lti_stability/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/01_lti_stability/generate_results.m');"
```

## 说明

- MATLAB 与 Python 输出的数值结果应保持一致。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
- `figures/01_lti_stability/lyapunov_stability_epsilon_delta.jpg` 属于示意图，不在脚本生成范围内。
