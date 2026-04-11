# 02 LTI Control Scripts

本目录用于复现 [`02_lti_control.md`](../../02_lti_control.md) 中的数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

## 示例系统

脚本采用如下二维系统：

$$
\dot x(t)=Ax(t)+Bu(t), \qquad y(t)=Cx(t)
$$

其中

$$
A=
\begin{bmatrix}
0.12 & 0.08 \\
-0.18 & -0.02
\end{bmatrix},
\quad
B=I_2,
\quad
C=
\begin{bmatrix}
1 & 0 \\
0 & 0.8
\end{bmatrix}.
$$

初始状态取为

$$
x(0)=
\begin{bmatrix}
1 \\
2
\end{bmatrix}.
$$

## 输出位置

- 图像：`figures/02_lti_control/`
- 数值结果：`generated/02_lti_control/`

## 运行方式

```powershell
python scripts/02_lti_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/02_lti_control/generate_results.m');"
```

## 说明

- MATLAB 与 Python 版本使用相同的系统矩阵和控制增益。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
