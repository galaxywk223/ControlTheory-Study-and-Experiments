# 控制理论学习与实验整理

这个仓库整理了我在现代控制理论方向上的学习笔记和配套数值实验。笔记以中文为主，实验分别提供 Python 和 MATLAB 版本，内容会继续补充。

## 内容概览

目前仓库包含以下主题：

1. [线性时不变系统稳定性](notes/01_线性时不变系统稳定性.md)
2. [线性时不变系统控制](notes/02_线性时不变系统控制.md)
3. [线性时不变系统周期采样控制与稳定性分析](notes/03_线性时不变系统周期采样控制与稳定性分析.md)
4. [鲁棒控制](notes/04_鲁棒控制.md)
5. [非线性时滞神经网络稳定性](notes/05_非线性时滞神经网络稳定性.md)
6. [混沌时滞神经网络同步与图像加密](notes/06_混沌时滞神经网络同步与图像加密.md)

这些笔记以连续时间 LTI 系统为起点，逐步延伸到采样控制、鲁棒控制，以及时滞神经网络和混沌同步问题。

## 实验说明

- `notes/` 保存笔记正文，按编号继续扩展。
- `scripts/` 保存复现实验脚本，按主题类别分组。
- `figures/` 保存笔记中直接引用的图像。
- `generated/` 保存脚本生成的数值结果，默认不纳入版本控制。

当前实验脚本分布如下：

- `scripts/foundations/`：线性系统稳定性、控制与周期采样控制
- `scripts/robust_control/`：鲁棒控制
- `scripts/nonlinear_and_delay/`：时滞神经网络稳定性、混沌同步与图像加密

## 运行方式

Python 依赖见 [requirements.txt](requirements.txt)。

在仓库根目录下可直接运行，例如：

```powershell
python scripts/foundations/01_lti_stability/generate_results.py
```

```powershell
python scripts/robust_control/04_robust_control/generate_results.py
```

MATLAB 版本同样在仓库根目录运行：

```powershell
matlab -batch "run('scripts/foundations/01_lti_stability/generate_results.m')"
```

```powershell
matlab -batch "run('scripts/nonlinear_and_delay/06_chaotic_sync_and_image_encryption/generate_results.m')"
```

## 仓库结构

```text
ControlTheory-Study-and-Experiments/
├─ notes/
├─ figures/
├─ scripts/
│  ├─ foundations/
│  ├─ robust_control/
│  └─ nonlinear_and_delay/
├─ generated/
├─ requirements.txt
├─ README.md
└─ LICENSE
```

## 说明

- 笔记和实验会继续补充，新内容仍按编号加入 `notes/`。
- 实验图像会尽量保持稳定的英文目录名，便于脚本和文档引用。
- `generated/` 中的文件主要用于本地复现检查，不作为仓库正文的一部分。

## 许可证

仓库中的笔记、图示与文档结构采用 [MIT 许可证](LICENSE) 开源。
