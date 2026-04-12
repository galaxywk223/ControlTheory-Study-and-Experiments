# 混沌时滞神经网络同步与图像加密笔记

## 混沌系统简介

混沌系统是指一类确定性的非线性动力系统。它本身不含随机项，但由于对初值极度敏感，长期行为会表现出极强的复杂性和不可预测性。

这里的关键矛盾在于：

- 系统是确定的，未来状态完全由初值和演化方程决定。
- 系统又是难以长期预测的，因为极小的初值偏差会被不断放大。

## 混沌时滞神经网络模型

考虑如下混沌时滞神经网络：

```math
\dot {x} (t) = A x (t) + B f (x (t)) + \bar {B} g (x (t - \tau)) + J. \qquad \text{(1)}
```

其中：

- $x(t) \in \mathbb{R}^n$ 是神经元状态向量。
- $B,\bar{B}$ 是连接权矩阵。
- $J$ 是输入常量。
- $\tau$ 是时滞。
- $f(\cdot), g(\cdot)$ 是激活函数。

假设激活函数满足 Lipschitz 条件，即存在正常数 $L_f, L_g$ 使得

```math
\|f(x_1)-f(x_2)\| \le L_f \|x_1-x_2\|, \qquad
\|g(x_1)-g(x_2)\| \le L_g \|x_1-x_2\|.
```

### 激活函数与 Lipschitz 条件说明

- 激活函数提供系统非线性，是复杂行为和混沌现象产生的根源之一。
- Lipschitz 条件限制函数变化率，保证模型“行为良好”，便于证明解的存在唯一性以及后续稳定性分析。
- 在混沌系统中，单次放大可能是温和的，但经过持续反馈和迭代后，仍会积累成显著差异。

## 驱动系统、响应系统与误差系统

将式 (1) 视为驱动系统，对应的响应系统取为

```math
\dot {\hat {x}} (t) = A \hat {x} (t) + B f (\hat {x} (t)) + \bar{B} g (\hat {x} (t - \tau)) + u(t) + J. \qquad \text{(2)}
```

定义同步误差

```math
e(t) = \hat{x}(t) - x(t),
```

可得误差系统

```math
\dot {e} (t) = A e (t) + B \left(f (\hat {x} (t)) - f (x (t))\right) + \bar {B} \left(g (\hat {x} (t - \tau)) - g (x (t - \tau))\right) + u (t). \qquad \text{(3)}
```

### 名词说明

- 驱动系统：主系统或参考系统，自主运行。
- 响应系统：从系统或受控系统，其目标是跟踪驱动系统。
- 误差系统：用于描述同步误差随时间的演化规律。

## 控制目标与控制器

设计如下形式的状态反馈控制器

```math
u(t) = K e(t), \qquad \text{(4)}
```

使得误差系统渐近稳定，即同步误差趋于零。

### 定义：渐进同步

若

```math
\lim_{t\to\infty} e(t)=0,
```

则称驱动系统和响应系统渐进同步。

## 关键引理

### 引理 1

对于适当维数的任意实矩阵 $X$ 和 $Y$，存在常数 $\varepsilon > 0$，使得

```math
X Y ^ {T} + Y X ^ {T} \leq \frac {1}{\varepsilon} X X ^ {T} + \varepsilon Y Y ^ {T}.
```

这就是后续证明中使用的杨氏不等式形式。

## 定理 1

若存在矩阵 $P > 0$、$Q > 0$、$M$ 以及标量 $\varepsilon$，使得

```math
\left[
\begin{array}{c c c c}
He\{PA+M\}+Q+\frac{1}{\varepsilon}L_f^2I & 0 & PB & P\bar{B} \\
* & -Q+\frac{1}{\varepsilon}L_g^2I & 0 & 0 \\
* & * & -\varepsilon I & 0 \\
* & * & * & -\varepsilon I
\end{array}
\right] < 0,
```

则系统 (1) 与系统 (2) 渐近同步，且控制增益可由

```math
K = P^{-1}M
```

求得。

### 证明思路

构造李雅普诺夫泛函

```math
V (e (t)) = e ^ {T} (t) P e (t) + \int_ {t - \tau} ^ {t} e ^ {T} (s) Q e (s) d s.
```

对其求导并代入误差系统，可得

```math
\dot {V} (e (t))
=
e ^ {T} (t) (He\{PA+PK\}+Q) e (t)
+ 2 e ^ {T} (t) P B \Delta f(t)
+ 2 e ^ {T} (t) P \bar{B} \Delta g(t-\tau)
- e ^ {T} (t - \tau) Q e (t - \tau),
```

其中

```math
\Delta f(t)=f(\hat{x}(t))-f(x(t)), \qquad
\Delta g(t-\tau)=g(\hat{x}(t-\tau))-g(x(t-\tau)).
```

再结合 Lipschitz 条件与杨氏不等式，对非线性项进行上界估计，最终得到一个关于
$e(t)$ 和 $e(t-\tau)$ 的二次型不等式。利用 Schur 补即可得到上面的 LMI 条件。

## 数值同步结果

### 系统设置

考虑如下两神经元时滞 Hopfield 网络：

```math
\left[
\begin{array}{l}
\dot{x}_1(t) \\
\dot{x}_2(t)
\end{array}
\right]
=
\left[
\begin{array}{c c}
-1 & 0 \\
0 & -1
\end{array}
\right]
\left[
\begin{array}{l}
x_1(t) \\
x_2(t)
\end{array}
\right]
+
\left[
\begin{array}{c c}
2 & -0.1 \\
-5 & 2
\end{array}
\right]
\left[
\begin{array}{l}
\tanh(x_1(t)) \\
\tanh(x_2(t))
\end{array}
\right]
+
\left[
\begin{array}{c c}
-1.5 & -0.1 \\
-0.2 & -1.5
\end{array}
\right]
\left[
\begin{array}{l}
\tanh(x_1(t-1)) \\
\tanh(x_2(t-1))
\end{array}
\right].
```

驱动系统与响应系统历史函数分别取为

```math
\left[
\begin{array}{c}
x_1(s) \\
x_2(s)
\end{array}
\right]
=
\left[
\begin{array}{c}
1.7 \\
2.5
\end{array}
\right],
\qquad
\left[
\begin{array}{c}
\hat{x}_1(s) \\
\hat{x}_2(s)
\end{array}
\right]
=
\left[
\begin{array}{c}
1 \\
2
\end{array}
\right],
\qquad
s \in [-1,0].
```

同步控制增益取为

```math
K =
\left[
\begin{array}{c c}
32.8175 & 47.7361 \\
-133.1490 & -79.0097
\end{array}
\right].
```

### 驱动系统相图

对驱动系统单独仿真并截取 $t \ge 20\,\mathrm{s}$ 的轨迹后，相平面轨迹如下：

![驱动系统相平面轨迹](figures/06_chaotic_sync/drive_phase_portrait.png)

轨线在平面内持续绕转并保持复杂形状，说明该系统本身具有明显的复杂动力学行为，适合作为同步与加密的密钥源。

### 同步误差收敛

误差状态与误差范数的变化如下：

![同步误差状态与误差范数](figures/06_chaotic_sync/synchronization_error.png)

由数值结果可见，

```math
\|e(t)\|_2 \le 10^{-2}
```

后不再离开的时间约为 $1.048\,\mathrm{s}$，

```math
\|e(t)\|_2 \le 10^{-4}
```

后不再离开的时间约为 $2.214\,\mathrm{s}$。控制输入两维的峰值绝对值分别约为 $46.8403$ 和 $132.7092$。在这组参数下，响应系统可以较快贴近驱动系统并保持同步。

## 图像加密结果

### 输入图像与通道分解

图像加密部分选取一幅 $640\times640$ 的 RGB 图像作为明文图像。

原图及其 RGB 通道如下：

![原图与 RGB 通道](figures/06_chaotic_sync/plaintext_rgb_channels.png)

同步轨迹在 $t=12\,\mathrm{s}$、$16\,\mathrm{s}$、$20\,\mathrm{s}$ 的状态值用于构造三个种子，可得

```math
s_1 \approx 0.6332,\qquad s_2 \approx 0.1481,\qquad s_3 \approx 0.3670.
```

接着用这组三个种子生成行置乱序列、列置乱序列以及逐像素扩散密钥流。

### 加密与解密结果

原图、密文图像与解密恢复结果如下：

![原图、加密图与解密图](figures/06_chaotic_sync/encryption_pipeline.png)

密文图像已经不再保留可辨识的视觉结构，而解密图像与原图逐像素一致。

### 通道直方图

原图与密文图像三个通道的像素直方图如下：

![原图与密文图像的 RGB 直方图](figures/06_chaotic_sync/rgb_histograms_before_after_encryption.png)

原图三个通道的分布都带有明显的内容特征；加密后各通道直方图明显变平，像素统计分布更接近均匀状态。

### 相邻像素相关性

灰度图像在水平与垂直方向上的相邻像素相关系数如下：

| 图像  | 水平相关系数  | 垂直相关系数  |
| --- | -------:| -------:|
| 原图  | 0.9854  | 0.9759  |
| 密文图 | -0.0103 | -0.0081 |

对应散点图如下：

![原图与密文图的相邻像素相关性散点图](figures/06_chaotic_sync/adjacent_pixel_correlation.png)

原图散点大多沿对角线分布，说明相邻像素高度相关；加密后的散点则扩散成近似云团，说明相邻像素关系已被有效打散。
