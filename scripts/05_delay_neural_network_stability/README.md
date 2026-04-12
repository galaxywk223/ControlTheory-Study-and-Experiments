# 05 非线性时滞神经网络稳定性脚本

本目录用于复现 [非线性时滞神经网络稳定性笔记](../../05_非线性时滞神经网络稳定性.md) 中的时滞神经网络稳定性数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

## 示例系统

脚本采用如下两神经元时滞网络：

$$
\dot x(t)=Ax(t)+Wf(x(t-\tau)),
\qquad
f(x)=\tanh(x),
$$

其中

$$
A=
\begin{bmatrix}
-1.0 & 0 \\
0 & -0.9
\end{bmatrix},
\qquad
W=
\begin{bmatrix}
0.35 & -0.28 \\
0.22 & 0.31
\end{bmatrix}.
$$

历史函数取为常值

$$
x(s)=
\begin{bmatrix}
1.2 \\
-1.0
\end{bmatrix},
\qquad
s \in [-\tau,0].
$$

## 输出位置

- 图像：`figures/05_delay_neural_network_stability/`
- 数值结果：`generated/05_delay_neural_network_stability/`

## 运行方式

```powershell
python scripts/05_delay_neural_network_stability/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/05_delay_neural_network_stability/generate_results.m');"
```

## 说明

- 脚本分别计算定理 1、定理 2、定理 3 的矩阵判定裕度。
- 时域仿真采用固定步长显式欧拉离散化，便于 MATLAB 与 Python 对齐。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
