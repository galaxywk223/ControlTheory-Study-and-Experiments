# 现代控制理论学习与实验笔记

本仓库当前公开内容聚焦于控制理论学习笔记，按主题整理了从线性系统基础到鲁棒控制、采样控制以及部分非线性问题的阅读与推导记录。

## 阅读顺序

建议按以下顺序阅读：

1. [线性时不变系统稳定性笔记](./01_线性时不变系统稳定性.md)
2. [线性时不变系统控制笔记](./02_线性时不变系统控制.md)
3. [线性时不变系统周期采样控制与稳定性分析笔记](./03_线性时不变系统周期采样控制与稳定性分析.md)
4. [鲁棒控制笔记](./04_鲁棒控制.md)
5. [非线性时滞神经网络稳定性笔记](./05_非线性时滞神经网络稳定性.md)
6. [混沌时滞神经网络同步与图像加密笔记](./06_混沌时滞神经网络同步与图像加密.md)

## 公开内容

- 线性时不变系统稳定性
- 线性时不变系统控制
- 线性时不变系统周期采样控制与稳定性分析
- 鲁棒控制
- 非线性时滞神经网络稳定性
- 混沌时滞神经网络同步与图像加密

## 复现实验脚本

相关脚本统一放在 `scripts/` 目录下，运行产生的数值结果统一放在 `generated/` 目录下。

当前已补充：

- `scripts/01_lti_stability/`：线性时不变系统稳定性笔记的 MATLAB/Python 脚本
- `scripts/02_lti_control/`：线性时不变系统控制笔记的 MATLAB/Python 脚本
- `scripts/03_periodic_sampling_control/`：线性时不变系统周期采样控制与稳定性分析笔记的 MATLAB/Python 脚本
- `scripts/04_robust_control/`：鲁棒控制笔记的 MATLAB/Python 脚本
- `scripts/05_delay_neural_network_stability/`：非线性时滞神经网络稳定性笔记的 MATLAB/Python 脚本
- `scripts/06_chaotic_sync_and_image_encryption/`：混沌时滞神经网络同步与图像加密笔记的 MATLAB/Python 脚本

对应输出：

- 图像写入 `figures/01_lti_stability/`
- 数值结果写入 `generated/01_lti_stability/`
- 图像写入 `figures/02_lti_control/`
- 数值结果写入 `generated/02_lti_control/`
- 图像写入 `figures/03_periodic_sampling/`
- 数值结果写入 `generated/03_periodic_sampling/`
- 图像写入 `figures/04_robust_control/`
- 数值结果写入 `generated/04_robust_control/`
- 图像写入 `figures/05_delay_neural_network_stability/`
- 数值结果写入 `generated/05_delay_neural_network_stability/`
- 图像写入 `figures/06_chaotic_sync/`
- 数值结果写入 `generated/06_chaotic_sync/`

运行示例：

```powershell
python scripts/01_lti_stability/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/01_lti_stability/generate_results.m');"
```

```powershell
python scripts/02_lti_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/02_lti_control/generate_results.m');"
```

```powershell
python scripts/03_periodic_sampling_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/03_periodic_sampling_control/generate_results.m');"
```

```powershell
python scripts/04_robust_control/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/04_robust_control/generate_results.m');"
```

```powershell
python scripts/05_delay_neural_network_stability/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/05_delay_neural_network_stability/generate_results.m');"
```

```powershell
python scripts/06_chaotic_sync_and_image_encryption/generate_results.py
```

```powershell
matlab -batch "cd('D:/Code/Lab/ControlTheory-Study-and-Projects'); run('scripts/06_chaotic_sync_and_image_encryption/generate_results.m');"
```

## 当前目录结构

```text
ModernControlTheory/
├─ figures/
├─ scripts/
│  ├─ 01_lti_stability/
│  │  ├─ README.md
│  │  ├─ generate_results.m
│  │  └─ generate_results.py
│  ├─ 02_lti_control/
│  │  ├─ README.md
│  │  ├─ generate_results.m
│  │  └─ generate_results.py
│  ├─ 03_periodic_sampling_control/
│  │  ├─ README.md
│  │  ├─ generate_results.m
│  │  └─ generate_results.py
│  ├─ 04_robust_control/
│  │  ├─ README.md
│  │  ├─ generate_results.m
│  │  └─ generate_results.py
│  ├─ 05_delay_neural_network_stability/
│  │  ├─ README.md
│  │  ├─ generate_results.m
│  │  └─ generate_results.py
│  └─ 06_chaotic_sync_and_image_encryption/
│     ├─ README.md
│     ├─ generate_results.m
│     └─ generate_results.py
├─ generated/                  # 运行脚本后自动生成，默认不纳入版本控制
├─ 01_线性时不变系统稳定性.md
├─ 02_线性时不变系统控制.md
├─ 03_线性时不变系统周期采样控制与稳定性分析.md
├─ 04_鲁棒控制.md
├─ 05_非线性时滞神经网络稳定性.md
├─ 06_混沌时滞神经网络同步与图像加密.md
├─ README.md
└─ .gitignore
```

## 开源协议

本仓库中的笔记、图示与文档内容基于 [MIT 许可证](./LICENSE) 开源。

补充说明：

- 本仓库的许可证仅覆盖当前仓库中自行编写和整理的笔记文本、图示素材与文档结构。
- 引用到的教材观点、论文结论、理论模型及其他第三方原始内容，不因本仓库采用 MIT 协议而自动转授任何额外权利。
