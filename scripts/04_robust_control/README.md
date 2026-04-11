# 04 Robust Control Scripts

本目录用于复现 [`04_robust_control.md`](../../04_robust_control.md) 中的区间不确定鲁棒控制数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

## 示例系统

脚本采用如下区间不确定系统：

$$
\dot x(t)=A(\rho_1,\rho_2)x(t)+Bu(t)+D\omega(t), \qquad z(t)=Ex(t),
$$

其中

$$
A(\rho_1,\rho_2)=
\begin{bmatrix}
0 & 1 \\
1.2+\rho_1 & 0.3+\rho_2
\end{bmatrix},
\qquad
\rho_1 \in [-0.6,0.6], \quad \rho_2 \in [-0.25,0.25],
$$

$$
B=D=
\begin{bmatrix}
0 \\
1
\end{bmatrix},
\qquad
E=
\begin{bmatrix}
1 & 0 \\
0 & 0.4
\end{bmatrix}.
$$

控制律取为

$$
u(t)=Kx(t),
\qquad
K=
\begin{bmatrix}
-3.9276 & -3.4536
\end{bmatrix}.
$$

## 输出位置

- 图像：`figures/04_robust_control/`
- 数值结果：`generated/04_robust_control/`

## 运行方式

```powershell
python scripts/04_robust_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/04_robust_control/generate_results.m');"
```

## 说明

- 脚本会扫描整个不确定区间上的闭环谱横坐标。
- 顶点处同时给出公共 Lyapunov 残差与频域增益估计。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
