# Yichao V3 → V9 基准与进度（2026-09-07）

本基准保存截至本次对话的用户决策、实际实现和证据。**离线链路已验证；真实输入、Yichao 实时决策及真机控制闭环尚未验收。** 用户已要求记录后继续推进部署；这不改变独立部署、保留他人运行程序和控制权交接的约束。

## 1. 已确认的版本与接口

- 用户确认：Yichao 最近发来的文字和截图未同步后续修改，具体接口以交付模型、配套 schema 和输入构造源码为准。不再等待与旧说明一致的新模型，不根据说明补零、删维或拼接旧动作。
- 本次重新查询远端 `refs/heads/yichao_plan`，与本地均为 `e14fd5ba3daa6327ac842d53fe5c4292b04e5716`；工作树干净。最新提交为 2026-09-06 03:13（北京时间）的 V3 190/199 交付。这里的“最新”仅指本次查询该远端分支的结果。见[查询记录](yichao_v3_v9/baselines/20260907_v3_199_offline/branch_verification.json)。
- 默认 actor **199**，190 仅作回归对照；启用 `safe_filter_v3`，不宣称 199 已证明真机更优。
- 下游固定 deploy `347890eb050a9b7f0856f311449d68f6411b6d92`，成套 V9 i19000 模型、sidecar、scheduler、mirror 和动作库；不适配 V10/V11。

| 接口 | 冻结模型对应内容 |
| --- | --- |
| Actor 输入 36D | 五帧双机相对球 XYZ 30D＋当前双机 canonical base XY 4D＋上一 shot 实际接受的目标 Y 2D |
| Actor 输出 2D | 每个 relay 一组双机目标 Y；由外围逻辑确定 hitter/non-hitter、锁存和交接 |
| Filter 状态 85D | 双机 base XY 五帧历史 20D＋双机 jointpos 58D＋击球点 XYZ、拍速 XYZ、TTS 共 7D |
| Filter 候选 2D | 当前候选目标，独立于 85D 状态；不是上一拍目标 |
| Filter 输出 | `head_probabilities[B,3,3]`、`member_risk[B,3]`、`conservative_risk[B]`；head 顺序为 base、inner hand、inner feet |

截图里的 `base_y history` 应按代码读作 `base_xy history`，每台五帧为 10D。两台各 29 个 jointpos 是实际测量输入；关节顺序与归一化参数属于预处理元数据，不是另一路传感器输入。36D actor 不含 jointpos。源码仍要求按实际 limits 归一化，不能直接将 SDK 原始顺序的 q 填入模型，也不能把夹具的 ±2 limits 用作真机参数。

训练槽位为 `left → 66/table_right/mirrored`、`right → 198/table_left/canonical`；现场共同坐标与朝向仍需用真实数据验收，不能按 ROS 的 left/right 字符串直连。球历史由训练源码的状态/速度/加速度和时间栅格构造，不能直接取最近五次 ROS 回调。

过滤阈值为 `0.3905482217669487`。现有实现使用 nominal＋7×7 候选网格＋home，网格半径为归一化动作单位 0.75；候选选择代价为平方距离加 `1e-3 * risk`，并有 0.45 m 间距保护。无安全候选则记录故障、禁止新 HIT；home 不自动算安全。归一化目标映射至 ±1.20 m。上一目标记忆不得直接使用 actor nominal。

依据：[交付 schema](../pingpang_planner/handoff/20260906_v3_planner_190_199/config/schema.json)、[交付部署说明](../pingpang_planner/handoff/20260906_v3_planner_190_199/DEPLOYMENT.md)、[特征构造源码](../pingpang_planner/doubles_planner/v0.py)。

## 2. 文件位置与身份

| 内容 | 位置 |
| --- | --- |
| 本地原始交付 | `/home/lyz/Desktop/code/PingPong/pingpang_planner/handoff/20260906_v3_planner_190_199/` |
| 本地独立适配包 | `/home/lyz/Desktop/code/PingPong/yichao_v3_v9/` |
| 工作站独立发布目录 | `/home/odl/codebase/yichao_v3_v9_e14fd5b/` |
| 工作站推理模型目录 | 上述发布目录下 `vendor/handoff/20260906_v3_planner_190_199/onnx/` |
| 默认 actor | `rl_planner_actor_model_199.onnx` |
| 安全过滤器 | `safe_filter_v3.onnx` |
| 原始训练 checkpoint | 原始交付下 `checkpoints/rl_planner_model_199.pt`、`checkpoints/safe_filter_v3.pt` |
| 工作站私有环境 | 发布目录下 `.venv/`；版本见该机 `requirements.lock` |
| 原 Fixed Runtime | `/home/odl/codebase/pingpang_doubles_v9_runtime`，仅作为基线，未覆盖其入口 |
| 文档指定 Predictor | `/home/odl/codebase/pingpang_predictor_dual_v9`；不能仅据目录推断实际进程来源 |

模型 SHA256：

```text
actor199  af83cc66238924595f24565bde0731c61ecc6808846132d7a98edd3d63bd62d7
actor190  5a28054c25b2a73d5ded7d5e8f5f5be767027d5837eec45168cf44a0adf5b01d
filter    96e2ea7c23cc134e35006723408f7727622366eea9a494f33718c8e8e1d9cbbe
```

工作站已验证回放 `output/runs/20260907T131214Z_9c308a63/` 对应发布 manifest SHA256 为 `661a967b74f5c0a479bee98af483fab25223ec51796629c8f943e7bf7cca957e`。这是该次运行身份，不代表此后现场文件永不变化。

## 3. 实现及验收边界

| 项目 | 已实现、已验证 | 未完成或限制 |
| --- | --- | --- |
| 推理和过滤 | CPU actor 199/190、过滤器、候选选择、解析保护；参考向量和 ONNX 对照测试 | 完整真实特征尚未验收 |
| Relay | 一拍一次决策、目标锁存、prepare、HIT/clear、RETURN、去重和超时 | 确认链路当前使用模拟 scheduler 反馈 |
| V9 执行 | 冻结 scheduler/mirror 离线消费；`v9_wire.py` 编码前整包检查 | 真实发布器和明确受理/完成 ACK 未实现 |
| 选择入口 | `run_planner.sh --planner fixed|yichao --mode ...` | 参数存在不代表 Yichao active 可用 |
| Yichao replay | 文件输入输出，可运行完整离线链路 | 不连接 ROS/LCM/设备 |
| observe | 现有输入只读诊断与日志 | 不运行 Yichao 决策，不构成 shadow 验收 |
| Yichao shadow/active | 当前明确报缺项退出 | 不会回退成 Fixed 或伪造可运行状态 |
| 原 Fixed shadow | 可选择原入口；仅做过 dry-run | 原入口也会创建 command 发布者，不能作为无控制输出的 observer |

工作站验证：9 项 selector/wire 测试＋21 项离线回归通过；249 帧合成双拍回放，66/198 各一次模拟 HIT，最终 COMPLETE，invalid/超期丢弃/队列丢弃均为 0，处理耗时 p50 0.948 ms、p95 1.011 ms、最大 3.190 ms。该时间不包含完整真实传感与网络链路，不能当作真机端到端延迟。

证据：[工作站验证](yichao_v3_v9/workstation_selector_verification.json)、[回放摘要](yichao_v3_v9/selector_replay.jsonl.summary.json)、[启动记录](yichao_v3_v9/selector_run.json)。本地一次回归出现双拍未完成，随后重试通过；原因未被证实，保留[初次记录](yichao_v3_v9/selector_offline_regression.json)和[重试记录](yichao_v3_v9/selector_offline_regression_retry.json)，不以重试抹掉失败。

## 4. 真实现场证据与仍缺的接入

- 用户说明：换电后两台机器人进入调试模式，此前描述为完全悬空；不是本任务验证过的实时安全状态。用户希望保留原程序，继续接入 Yichao 并记录真实击球；不要求重新测试 Predictor 算法功能。
- 已在两台机器人分别用只读 DDS subscriber 采集各 29D q/dq 和 IMU；记录中是 SDK 顺序，尚未成为持续的 Planner 输入。采集程序不发布 LowCmd，限时退出。初次 66 的 SDK 析构报错和修复后两台退出结果均保留在[原始记录目录](yichao_v3_v9/live_diagnostic_20260907T124508Z/)。这些记录不证明换电后的持续输入可用。
- 已从工作站看到用户拿球移动的原始动捕位置变化，记录在[球移动采样](yichao_v3_v9/ball_movement_20260907T124933Z.jsonl)。采样中的源时间比工作站 wall time 约快 112.14 秒；时钟域未核清，不能把新接收时间当作新采样时间。
- 2026-09-07 21:28:28（北京时间）ROS master 查询存在单路球预测、拍速、`/torso_pose_origin` 注册发布者；五个双打适配输入（ball prediction、双机 state、双机 torso）没有注册发布者。该查询不证明注册节点仍实时发布。两机器人进程名筛查未匹配到 `deploy_policy.py/g1_control`，也不证明电机无输出或设备可接管。见[当时的只读快照](yichao_v3_v9/current_input_audit_20260907T132828Z.json)。
- 此前实际 Predictor 来源是 `/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/TableTennis.py --rigid-id 0`，与手册双打目录不同。已有球流可作接入来源，不将 topic 名不同直接等同于 Predictor 故障；要补字段映射与双机 torso 来源。

下一步按顺序推进：双机 q 持续采集与 SDK→训练顺序转换；双机共同坐标 base/torso；现有球预测字段与时钟映射；严格 36D/85D 历史和初始目标记忆；真实输入只读推理；专用命令发布、受理/完成 ACK 与真实目标反馈；满足现场窗口和保护条件后分步 active。输入不足时记录具体缺项，不以合成值通过验收。

当前 `inputs.py` 对 `provenance=real` 明确拒绝，`launch.py` 的 Yichao shadow/active 明确关闭。需要完成实现和验收后再打开，不能只删这两处拒绝条件。训练软限位、实际历史节拍和精确 Controller 未提交补丁仍需从可追溯资产核对；训练元数据导出工具只是备选辅助，不代表必须让用户重新提供 jointpos。

## 5. 保存内容与回退方法

[基准目录](yichao_v3_v9/baselines/20260907_v3_199_offline/) 包含：

- `baseline.json`：机器可读的模型选择、维度、路径和未完成状态。
- `branch_verification.json`：本次远端与本地版本查询。
- `local_adapter_and_planner_snapshot.tar.gz`：50 个本地文件，含适配源码、测试、夹具、配置、选定辅助工具、启动入口、Planner ONNX/TorchScript 和 schema。
- `snapshot_manifest.json`：压缩包和每个文件 SHA256；已逐成员校验。

这个归档是本地适配与 Planner 模型快照，**不是整套工作站环境或机器人部署备份**：不包含 `.venv`、完整 vendor Controller/Deploy、V9 底层权重及全部动作数据。其位置和身份仍依赖已有固定副本与发布清单。归档内 `requirements.lock` 和 `config/assets.lock.json` 来自本地，不能覆盖工作站的同名文件；工作站私有 ROS 模块与依赖必须使用该机版本。

回退先核对 `snapshot_manifest.json` 的压缩包 SHA256，解压到一个新的独立目录，再核对每个成员。根据对应机器的固定资产和依赖补齐运行目录，重新运行离线测试和回放；不要直接覆盖正在运行的发布目录或原 Fixed Runtime。完整机载回滚仍须成套 V9 资产和控制权交接，不能只替换 actor/filter。

根目录 `PingPong` 不是 Git 仓库；此记录不声称已经生成 Git commit/tag。归档不包含根 `AGENTS.md` 或任何登录凭据。后续改动另存新基准，不覆盖本基准。机器人已经执行的动作不能靠撤回消息或恢复目录撤销；本任务从未启动 Yichao 真机控制，不能把恢复该快照描述为已验证的物理回滚。
