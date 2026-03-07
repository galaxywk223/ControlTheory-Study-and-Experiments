# 23_wu_zhou1_amm

本目录记录论文
`Event-triggered extended dissipativity stabilization of semi-Markov switching systems`
的复现工作，内容包括问题整理、公式推导、DC 电机算例实现、脚本说明与待完成事项。

## 当前进度

- 已完成：论文问题与模型整理。
- 已完成：事件触发机制和两阶段闭环形式整理。
- 已完成：生成元、Lyapunov-Krasovskii 泛函和 LMI 推导主线梳理。
- 已完成：DC 电机算例的原型实现与结果出图。
- 已完成：MATLAB/Python 脚本分组整理。
- 待完成：论文结果对照、脚本命名统一、主线与扩展线进一步分离。

## 阅读顺序

1. [`notes/01_论文概览与复现目标.md`](./notes/01_论文概览与复现目标.md)
2. [`notes/02_系统模型与事件触发机制.md`](./notes/02_系统模型与事件触发机制.md)
3. [`notes/03_生成元与LMI推导整理.md`](./notes/03_生成元与LMI推导整理.md)
4. [`notes/04_DC电机算例复现记录.md`](./notes/04_DC电机算例复现记录.md)
5. [`notes/05_脚本说明与执行顺序.md`](./notes/05_脚本说明与执行顺序.md)
6. [`notes/06_当前遗留问题.md`](./notes/06_当前遗留问题.md)

## 目录结构

```text
23_wu_zhou1_amm/
├─ README.md
├─ notes/
├─ assets/
│  └─ figures/
└─ scripts/
   ├─ matlab/
   │  ├─ legacy/
   │  └─ zie_pipeline/
   └─ python/
```

## 内容边界

- 仓库公开内容：复现笔记、实现脚本、数值实验图。
- 仓库不包含原论文正文或其转存版本。
