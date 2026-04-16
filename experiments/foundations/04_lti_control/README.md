# 04 线性时不变系统控制实验

本目录复现 [`04_线性时不变系统控制`](../../../notes/04_线性时不变系统控制.md) 中的开环与闭环响应，对应状态反馈和输出反馈两类控制器设计。

## 关联笔记

- [04_线性时不变系统控制](../../../notes/04_线性时不变系统控制.md)

## 实验内容

- 生成开环状态响应。
- 比较多组状态反馈控制结果。
- 比较多组输出反馈控制结果并导出配套数值报告。

## 代表结果

闭环响应图用于对比控制器引入前后的状态收敛差异。

<p align="center">
  <img src="../../../figures/04_lti_control/theorem_3_state_feedback_response.png" alt="线性时不变系统状态反馈闭环响应" width="760" />
</p>

## 运行命令

Python 依赖见 [requirements.txt](../../../requirements.txt)。以下命令在仓库根目录执行。

```bash
python experiments/foundations/04_lti_control/generate_results.py
matlab -batch "run('experiments/foundations/04_lti_control/generate_results.m')"
```

## 输出目录

- 图像：`figures/04_lti_control/`
- 数值结果：`generated/04_lti_control/`
- `generated/` 默认只用于本地复现检查，不纳入版本控制。

## 代码入口

| 路径 | 作用 |
| --- | --- |
| `generate_results.py` | Python 版开环与闭环结果生成入口 |
| `generate_results.m` | MATLAB 版结果生成入口 |
