# 06 Chaotic Sync And Image Encryption Scripts

本目录用于复现 [`06_chaotic_sync_and_image_encryption.md`](../../06_chaotic_sync_and_image_encryption.md) 中的同步与图像加密数值结果。

## 内容

- `generate_results.py`：Python 版本
- `generate_results.m`：MATLAB 版本

## 示例系统

脚本采用文中的两神经元时滞 Hopfield 网络：

$$
\dot x(t)=Ax(t)+B\tanh(x(t))+\bar B\tanh(x(t-\tau)),
\qquad
\tau=1,
$$

其中

$$
A=
\begin{bmatrix}
-1 & 0 \\
0 & -1
\end{bmatrix},
\quad
B=
\begin{bmatrix}
2 & -0.1 \\
-5 & 2
\end{bmatrix},
\quad
\bar B=
\begin{bmatrix}
-1.5 & -0.1 \\
-0.2 & -1.5
\end{bmatrix}.
$$

同步控制增益取为

$$
K=
\begin{bmatrix}
32.8175 & 47.7361 \\
-133.1490 & -79.0097
\end{bmatrix}.
$$

图像加密部分使用仓库内的输入图像：

- `figures/06_chaotic_sync/plaintext_input_image.jpg`

## 输出位置

- 图像：`figures/06_chaotic_sync/`
- 数值结果：`generated/06_chaotic_sync/`

## 运行方式

```powershell
python scripts/06_chaotic_sync_and_image_encryption/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/06_chaotic_sync_and_image_encryption/generate_results.m');"
```

## 说明

- 同步仿真采用固定步长显式 Euler 离散化。
- 图像加密先做行列置乱，再做逐像素扩散，解密会检查是否精确恢复原图。
- 图像文件使用语义化文件名，便于直接在仓库中引用。
