# 06 混沌时滞神经网络同步与图像加密实验

本目录用于复现 [`06_混沌时滞神经网络同步与图像加密`](../../../notes/06_混沌时滞神经网络同步与图像加密.md) 中的同步控制和图像加密结果。

## 生成内容

- 驱动系统相图
- 同步误差与控制输入结果
- 原图、密文图和解密图
- 直方图与相邻像素相关性统计

## 运行方式

在仓库根目录执行：

```powershell
python scripts/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/generate_results.py
```

```powershell
matlab -batch "run('scripts/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/generate_results.m')"
```

## 输出位置

- 图像：`figures/06_chaotic_sync_and_image_encryption/`
- 数值结果：`generated/06_chaotic_sync_and_image_encryption/`

## 依赖

- Python：`numpy`、`matplotlib`、`Pillow`
- MATLAB：基础数值与图像处理函数
