# 01 系统建模与状态空间基础实验

本目录复现 [`01_系统建模与状态空间基础`](../../../notes/01_系统建模与状态空间基础.md) 中的质量-弹簧-阻尼系统建模结果，重点展示状态空间表达、平衡点和输入驱动响应。

## 关联笔记

- [01_系统建模与状态空间基础](../../../notes/01_系统建模与状态空间基础.md)

## 实验内容

- 从质量-弹簧-阻尼参数构造状态空间模型。
- 生成自由响应下的状态与输出结果。
- 比较零输入与单位常值输入下的强迫响应并导出数值报告。

## 代表结果

状态与输出响应图把状态变量选择和时域响应联系在同一个模型里。

<p align="center">
  <img src="../../../figures/01_state_space_modeling/state_output_response.png" alt="系统建模与状态空间基础实验结果" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。以下命令在仓库根目录执行。

```bash
python experiments/foundations/01_state_space_modeling/generate_results.py
matlab -batch "run('experiments/foundations/01_state_space_modeling/generate_results.m')"
```

## 输出目录

- 图像：`figures/01_state_space_modeling/`
- 数值结果：`generated/01_state_space_modeling/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版状态空间结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
