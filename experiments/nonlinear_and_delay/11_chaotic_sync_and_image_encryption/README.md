# 11 混沌时滞神经网络同步与图像加密实验

本目录复现 [`11_混沌时滞神经网络同步与图像加密`](../../../notes/11_混沌时滞神经网络同步与图像加密.md) 中的同步控制和图像加密结果，把控制同步和图像处理放在同一条实验链路里。

## 关联笔记

- [11_混沌时滞神经网络同步与图像加密](../../../notes/11_混沌时滞神经网络同步与图像加密.md)

## 实验内容

- 生成驱动系统相图和同步误差结果。
- 生成原图、密文图和解密图。
- 输出直方图与相邻像素相关性统计。

## 代表结果

图像加密流程图把同步轨迹、加密结果和解密结果串成一条完整链路。

<p align="center">
  <img src="../../../figures/11_chaotic_sync_and_image_encryption/encryption_pipeline.png" alt="混沌同步与图像加密流程图" width="920" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。以下命令在仓库根目录执行。

```bash
python experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/generate_results.py
matlab -batch "run('experiments/nonlinear_and_delay/11_chaotic_sync_and_image_encryption/generate_results.m')"
```

## 输出目录

- 图像：`figures/11_chaotic_sync_and_image_encryption/`
- 数值结果：`generated/11_chaotic_sync_and_image_encryption/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版同步与图像加密结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
