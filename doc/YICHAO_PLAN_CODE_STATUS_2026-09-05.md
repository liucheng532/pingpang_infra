# yichao_plan 代码现状与迁移阅读地图

检查日期：2026-09-05。范围仅为本地 `pingpang_planner`，分支 `yichao_plan`，提交 `d51db57a562df0d9d00b74677406c9ce3bb8e9da`。本次检查代码、交付配置和已有验证记录，并执行交付包 SHA256 校验；未重新训练、运行 Isaac、执行真机测试或修改远程工作站。未 fetch，不声明这是 GitHub 最新提交。

## 核心结论

本提交最新交付是 **V1 shot-level Frozen-CBF + RL Planner**。已有 actor、安全风险模型、推理导出、仿真训练/评估入口和部署说明，但不能只复制 actor 就认为完成真机迁移。仍需接入真实观测、候选筛选、生命周期、控制器协议和通信。

最新交付入口为 [20260903 V1 README](../pingpang_planner/handoff/20260903_v1_frozen_cbf_rl/README.md) 和 [DEPLOYMENT](../pingpang_planner/handoff/20260903_v1_frozen_cbf_rl/DEPLOYMENT.md)。根目录 `CURRENT_PLANNER_USAGE.md` 描述的 241D/210D 路线、8 月 27 日交付的 294D/12D 路线与最新 V1 不同，不能混用输入或 checkpoint。

## 当前 V1 数据流

```text
球历史、双机 base、上一轮执行目标
  → 36D 观测预处理 → actor → 2D nominal 横向目标
双机 base 历史、关节位置、击球目标等
  → 87D 安全观测 + 2D 候选动作 → 冻结风险模型
  → 候选筛选 / 无候选时 home 回退
  → 解析间距约束 → 单球生命周期目标保持及协议门控
  → 双机 base 目标 → 下层控制器
```

V1 actor **只输出左右两台机器人的横向 Y 目标**，不直接输出击球点、拍面或关节指令，也没有独立的“选择击球机器人”输出。击球归属、准备/击球/回位及交接由外围 relay/protocol 逻辑承担。

| 接口 | 内容 |
| --- | --- |
| actor 输入 36D | 5 帧球历史相对左右 base 的 XYZ（30D）+ 双机当前 base XY（4D）+ 上次双机目标 Y（2D） |
| actor 输出 2D | 左右机器人归一化 Y 目标；按训练 workspace `[-0.95, 0.95] m` 映射 |
| CBF 输入 87D | 双机 5 帧 base XY（20D）+ 双机 29 维关节位置（58D）+ 击球位置（3D）+ 拍速（3D）+ 击球剩余时间（1D）+ nominal 双机命令（2D） |
| CBF 额外输入 | 待评估候选动作 2D |
| CBF 输出 | 3-member ensemble 对 base、inner hand、inner feet 三类距离风险评分；保守风险用于候选接受判断 |

字段顺序及 hash 以 [schema.json](../pingpang_planner/handoff/20260903_v1_frozen_cbf_rl/config/schema.json) 为准。物理量预处理仍需在调用方完成；actor 导出已经包含 empirical normalizer，不能重复执行这层归一化。

## 代码阅读顺序

| 文件 | 作用 |
| --- | --- |
| `doubles_planner/v1.py` | V1 公共名称，主要重导出 v0 模块实现；看到 v0 名称不代表必须用旧模型 |
| `doubles_planner/v0.py` | V1 schema、观测构造、冻结风险网络与过滤器、解析间距投影及 `IsaacV1ShotPlannerEnv` 单球执行循环 |
| `doubles_planner/core.py`、`protocol.py`、`models.py` | Relay 规划、协议和球/机器人反馈/命令数据结构 |
| `doubles_planner/real_inputs.py` | 真实输入适配及 freshness/缺失检查，是理解真机输入契约的参考 |
| `doubles_planner/deployment.py` | 传统 real-planner 工厂；`rl` 仍是 `FrozenQPolicy` 路线，不能把它当作加载最新 V1 actor 的开关；默认 student profile 仍为 V6 |
| `train_isaac_doubles_ppo.py`、`evaluate_isaac_doubles_ppo.py` | Isaac 训练与复评入口 |
| `scripts/export_v1_handoff_models.py` | V1 actor / barrier 导出 |

`IsaacV1ShotPlannerEnv` 依赖仿真环境状态和步进，不能直接替换成真机节点。虽然已有 real-input adapter，但已检查的 real-planner 工厂没有串接 V1 actor 和冻结 CBF 的完整路径。

## 模型与过滤逻辑

交付目录：`pingpang_planner/handoff/20260903_v1_frozen_cbf_rl/`。

- 选定模型：`checkpoints/v1_frozen_cbf_rl_model_100.pt`。
- actor：`onnx/v1_planner_actor.onnx`，另有 TorchScript 导出。
- 风险模型：`checkpoints/frozen_v1_barrier.pt` 与 `onnx/frozen_v1_barrier_risk.onnx`。
- 候选：7×7 偏移网格加 nominal，共 50 个（clamp 后可能重合）；过滤间距不足和风险超过阈值的候选，再按偏离 nominal 的代价选择。
- 风险接受阈值：`0.552091258764267`；无可接受候选时回退到物理 Y `[-0.35, 0.35] m`。
- 之后仍有解析投影，目标物理 Y 间距至少 `0.45 m`，并在单球生命周期内保持目标。ONNX scorer 本身不包含这些外围逻辑。

## 已有证据和本次验证边界

本次 `sha256sum -c SHA256SUMS` 的 17 个清单条目全部通过。以下性能数值来自交付记录，未在本机重新复现：

- actor/CBF ONNX 在 batch 1、7、32 上的数值一致性记录为通过。
- model 100 的 24-shot 仿真：1 collision、1 unsafe；平均击球位置误差约 0.161 m、拍速误差约 0.904 m/s、姿态误差约 0.219 rad、时间误差约 10.13 ms。
- model 109 同 seed 小样本记录为 0 collision、0 unsafe，但不能替代 model 100 的证据。正式四路对比 `v1_metrics.json` 使用的是 109。
- 103 只有训练曲线记录，没有保存 checkpoint。选 100 是交付中记录的既有决定，不是本次重新选型。

因此当前状态是“已有可校验的模型和仿真交付”，尚不能称为真机验证通过。

## 为后续迁移需要明确的接口

1. 从真实 Predictor/遥测构造同顺序、同坐标系、同时间语义的 36D/87D 输入；核对球历史和 base 历史采样方式、关节顺序及关节归一化。
2. 在真机运行循环中接入 actor、完整候选过滤、回退和解析约束，而非只调用两个 ONNX 文件。
3. 对齐 hitter 归属、phase/ready、何时更新目标、何时保持目标和何时交接；将最终目标送入正确的机器人控制接口。
4. 对齐训练时的 WBC/controller contract、workspace、左右镜像及标定。交付记录使用 V9 i19000 student、指定 controller revision 和外部 motion banks，不能从现场目录名推定兼容。
5. 在实际接线后验证输入过期/缺失、候选不可行、时间延迟和安全保持行为，再进行 shadow 验收。

本窗口当前仅分析源分支；以上是源代码暴露的迁移需求，不是对工作站二目标代码完成差异审计后的结论。
