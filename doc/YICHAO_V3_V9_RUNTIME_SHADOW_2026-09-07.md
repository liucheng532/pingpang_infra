# V3 实时接口接入：诊断 shadow

本轮已把现有 ROS 消息接到 V3 特征、actor 199、安全过滤和 V9 移动命令预览，并同步工作站独立目录 `/home/odl/codebase/yichao_v3_v9_e14fd5b`。这是只读诊断接口，不代表完整真实输入或执行闭环已验收。

## 实现与复用范围

- `src/yichao_v3_v9/runtime_shadow.py` 消费原 V9 `/doubles/table_right/state`（66）与 `/doubles/table_left/state`（198）；直接使用其中的 Lab 顺序 q、pelvis、phase 和当前目标。关节归一化复用用户指定 URDF。
- 优先消费已有 `/doubles/ball_prediction`；该发布者不存在时选用现有 `/table_tennis_planner_monitor`。一轮运行固定一个球来源，不混合两个 shot 流。
- 将两路 base 历史插值到 20 ms 五点栅格，拒绝启动历史不足、超过 50 ms 的历史缺口/配对差、旧数据和未来数据；不取最近五次回调，不补零。
- 36D/85D 向量对照冻结训练源码逐元素通过；拍速与球速分开，关节物理量与归一化量分开。
- 每拍只执行一次 actor＋冻结安全过滤，失败或超时不生成移动预览；后续同拍预测不会重新触发决策。
- `v9_wire.encode` 复用现有 V9 `stage` 横向移动入口。Fixed Planner 的 `hold` 为 no-op，不能用来承载 learned target。
- 命令预览的 `valid=False`、`active=False`，只写文件。测试中在纯离线 bridge 对象上将副本设为有效，验证目标被转交 `set_external_base_target`；原预览传给 bridge 不产生调用。没有构造 ROS 控制发布者。

该模块尚未替换原 Fixed Relay 的完整击球/回位状态机，也没有解除 active 门禁。原 Runtime 自带 token/phase 逻辑可复用；其固定 home/outward 目标覆盖、Yichao 目标记忆和执行受理关联仍需接入。此处不能把“移动命令预览已连通”写成“完整 learned relay 已上机”。

## 诊断假设与真实验收边界

所有推理记录保留 `real_input_accepted=False` 和 `command_ack=False`，并记录以下假设：

1. 历史时间轴目前为工作站接收时间；机器人源 monotonic、Predictor 源时间另行保留，未宣布传感器源龄或跨机时钟对齐通过。
2. 动捕 origin 暂作训练共同坐标的诊断候选；朝向、标定的物理验收仍待完成。
3. 球历史用训练初始化的零加速度模型重建，未声称这是实测球加速度。
4. actor 的 previous target 使用原控制器当前报告的目标作对照输入，日志明确它不是“上一拍 Yichao 实际接受的目标”。模型 nominal 和 preview 都不会回写该输入。

这些假设只用于观察真实输入下模型会给出什么建议，不能因格式/参考向量测试通过就启用 active，也不能用原控制器 phase 评价 Yichao 闭环效果。

## 启动入口

在工作站独立目录运行：

```bash
./run_planner.sh --planner yichao --mode shadow --subscriber-ip 192.168.123.165
```

默认 CPU、受限线程、20 秒、唯一节点名，关闭 rosout 发布，输出在新的 `output/runs/<run_id>/shadow.jsonl`。启动器会标明 `diagnostic_shadow=true`。`--mode active` 仍拒绝启动。

原始消息可离线重放：

```bash
.venv/bin/python -B tools/replay_runtime_shadow.py --input recording.jsonl --output shadow_replay.jsonl
```

如录制的是原双打预测消息，增加 `--ball-topic /doubles/ball_prediction`。不同运行的状态与球记录不能跨时段拼接。

## 验证与现场结果

- 本地完整测试：83 项通过。新增 runtime 测试 11 项，含冻结源码向量对照、非均匀采样、缺历史、重复/重启/发布者变更、非法四元数、急停/transport error、超时/无安全候选、两种 Predictor 消息和真实 ONNX 合成输入。
- 工作站 runtime 11 项和 selector/wire 10 项通过；最终工作站完整 83 项测试也通过（2.723 秒），7 个同步源码/测试/工具文件哈希与本地一致。最终工作站 manifest SHA256 为 `d90562cd89b18ec0fc41614adc4517c158748e22ef62b02f9a82f7de37ba3399`。
- 只读运行 `output/runs/20260907T150718Z_40c5aabc` 正常退出：20 秒收到 monitor 回调 583 次，约 29.14 Hz，队列丢弃 0，发布者变更 0。
- 两路 `/state` 当时均无发布者、无消息；运行期决策 0，控制命令 0。不能称为完整真实输入推理成功。
- 随后 SSH 只读检查两台机器人，未发现 `deploy_policy.py`、`g1_control` 或本任务 joint reader。此进程快照不证明零力矩或已完成控制权交接。
- 工作站和本机 wall time 存在偏差；目录名分别由各机时间生成，不能直接以目录时间相减作为链路延迟。

证据目录：[runtime_shadow_integration](yichao_v3_v9/runtime_shadow_integration/)，含代码同步、测试、原始采集、ROS/进程检查和工作站 manifest。现场原 Predictor、Runtime、机载部署目录和进程未修改或启停。

## 回退与下一步

**2026-09-08 更新：** 已将 `target_feedback.py` 与对应测试同步到工作站独立目录。它把 session、sequence、token、reported target、phase、位置和速度逐项关联；单独看到 `last_applied_sequence` 不会被标记为 ACK。工作站和机器人当前进程复查均为空，本任务没有后台进程。

更新前的独立方案文件保存在工作站 `output/runtime_shadow_20260907T150842Z/before.tar.gz`；压缩包是本轮已有文件的备份，不含新增文件或完整依赖/资产。原始完整基准归档保持不变。本轮 observer 已自行退出，无需处理他人进程。

下一步按 V9 手册核对机载现有入口和状态发布路径，再接通两路 `/state`。当前只处于机器人调试模式时，ROS 不会自动产生上述状态消息。启动机载程序涉及电机输出，不能为了出现 `/state` 随意启动控制器；先明确本轮启动动作和现场配合，再继续真实输入及移动专项验收。
