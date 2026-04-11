# ModernControlTheory

本仓库当前公开内容聚焦于控制理论学习笔记，按主题整理了从线性系统基础到鲁棒控制、采样控制以及部分非线性问题的阅读与推导记录。

## Reading Order

建议按以下顺序阅读：

1. [`01_lti_stability.md`](./01_lti_stability.md)
2. [`02_lti_control.md`](./02_lti_control.md)
3. [`03_periodic_sampling_control.md`](./03_periodic_sampling_control.md)
4. [`04_robust_control.md`](./04_robust_control.md)
5. [`05_delay_neural_network_stability.md`](./05_delay_neural_network_stability.md)
6. [`06_chaotic_sync_and_image_encryption.md`](./06_chaotic_sync_and_image_encryption.md)

## Public Contents

- 线性时不变系统稳定性
- 线性时不变系统控制
- 周期采样控制与稳定性分析
- 鲁棒控制
- 非线性时滞神经网络稳定性
- 混沌同步与图像加密

## Reproduction Scripts

相关脚本统一放在 `scripts/` 目录下，运行产生的数值结果统一放在 `generated/` 目录下。

当前已补充：

- `scripts/01_lti_stability/`：线性时不变系统稳定性笔记的 MATLAB/Python 脚本

对应输出：

- 图像写入 `figures/01_lti_stability/`
- 数值结果写入 `generated/01_lti_stability/`

运行示例：

```powershell
python scripts/01_lti_stability/reproduce.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/01_lti_stability/reproduce.m');"
```

## Current Layout

```text
ModernControlTheory/
├─ figures/
├─ scripts/
│  └─ 01_lti_stability/
│     ├─ README.md
│     ├─ reproduce.m
│     └─ reproduce.py
├─ generated/                  # 运行脚本后自动生成，默认不纳入版本控制
├─ 01_lti_stability.md
├─ 02_lti_control.md
├─ 03_periodic_sampling_control.md
├─ 04_robust_control.md
├─ 05_delay_neural_network_stability.md
├─ 06_chaotic_sync_and_image_encryption.md
├─ README.md
└─ .gitignore
```

## 开源协议

本仓库中的笔记、图示与文档内容基于 [MIT License](./LICENSE) 开源。

补充说明：

- 本仓库的许可证仅覆盖当前仓库中自行编写和整理的笔记文本、图示素材与文档结构。
- 引用到的教材观点、论文结论、理论模型及其他第三方原始内容，不因本仓库采用 MIT 协议而自动转授任何额外权利。
