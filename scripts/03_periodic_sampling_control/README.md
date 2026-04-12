# 03 线性时不变系统周期采样控制脚本

本目录用于复现 [线性时不变系统周期采样控制与稳定性分析笔记](../../03_线性时不变系统周期采样控制与稳定性分析.md) 中的周期采样控制数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

## 示例参数

脚本采用笔记中的参数：

$$
\alpha = 0.05,\qquad h = 0.125,
$$

$$
K=
\begin{bmatrix}
-0.3399 & -0.0263 \\
-0.0628 & -1.2527
\end{bmatrix},
$$

$$
A=
\begin{bmatrix}
-0.479908 & -3.81625 \\
5.1546 & 14.4723
\end{bmatrix},
\qquad
B=\operatorname{diag}(5.8705212,15.50107),
$$

$$
x(0)=
\begin{bmatrix}
1 \\
-1
\end{bmatrix}.
$$

## 输出位置

- 图像：`figures/03_periodic_sampling/`
- 数值结果：`generated/03_periodic_sampling/`

## 运行方式

```powershell
python scripts/03_periodic_sampling_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/03_periodic_sampling_control/generate_results.m');"
```

## 说明

- 脚本采用零阶保持采样控制律 `u(t)=Kx(t_k)`。
- MATLAB 与 Python 版本使用相同的系统矩阵和控制增益。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
