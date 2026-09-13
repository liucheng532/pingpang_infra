# V6R10 model630 actor-only / V11 i42500

这是与现有 `yichao_v3_v9` 并列的独立部署。它复用原
`FixedRelayPlanner` 的来球预留、hitter、commit token、HIT 和回位状态机，
只把每拍的双机 lateral-Y proposal 换成 V6R10 `model_630` 的联合 actor。
旧 V3+V9 目录、入口、ROS 11312、Fixed、Predictor、Controller 和共享标定均不覆盖。

当前软件已完成离线、工作站 shadow、V11 Controller shadow 和 live
transport-shadow 双 ACK 验证。它具备进入 active 物理测试的软件条件，但这不代表
R2、站稳、击球或回球已经通过；这些仍以当次现场结果为准。

## 固定身份与模型 ABI

| Actor 槽位 | 球台身份 | ROS 身份 | 物理主机 | 输出 |
|---|---|---|---|---|
| left | table_left | `table_left` | 198 | `action[0]` |
| right | table_right | `table_right` | 66 | `action[1]` |

`models/model630_actor.onnx` 是从 handoff 的 `model_630.pt` 重建的确定性 actor
mean：`36→256→256→2`、ELU、冻结 mean/std，归一化公式是
`(observation - mean) / (std + 0.01)`。ONNX 支持动态 batch；生产只加载 ONNX
Runtime，不加载 PyTorch checkpoint。sidecar 固定 checkpoint/schema/ONNX hash、
36D actor schema、87D audit schema 和 PT/ONNX parity。最大导出误差为
`2.384185791015625e-06`，门槛为 `2e-5`。

36D 顺序严格来自 handoff：5 帧球相对 left/right base、两机当前 XY、上一组完整
双 ACK 目标。base Y 除数是 `1.0`，previous target 按 `0.9 m` 归一化，初值为
`[+0.35,-0.35] m`。87D audit 含双机 base history、58D joint position、7D
strike、hitter sign 和 interval；它只记录，不送入 actor，也不触发 barrier。

首球 interval 为 `1.65 s`。只有不同 shot 的双侧 staging 完整 ACK 才原子更新
previous target 和接收时刻；后续间隔裁剪到 `[1.5,1.8] s`，再用
`(interval-0.8)/0.8`。proposal、单侧 ACK、重复/无效/fallback shot 均不更新。

## Actor-only guard

生产目录不含 frozen learned barrier、risk ensemble、51×51 搜索或 safe filter。
actor 输出先裁剪到 `[-1,1]`，再乘 `0.9 m`。若
`left_target - right_target < 0.50 m`，固定 Fixed 已选 hitter 的目标，只将 peer
向外投影到恰好 `0.50 m`。workspace 无法容纳、非有限数、维数错误或投影后检查
失败时，该 `shot_id` 永久拒绝并请求两机规则 home。日志固定写
`filter_mode=off`，barrier/risk 写 `not_evaluated`，不会用 0 冒充评分。

## Fixed relay 与 ACK

Fixed 首次为 shot 产生 reservation/token 后，actor pair 按该 token 锁存。Planner
向两侧发送 V11 已支持的 `role=stage`，ACK 只检查：

- `last_planner_session_id` 等于当前 session；
- `last_applied_sequence` 不小于本侧 staging sequence；
- `transport_error` 为空；
- `target_base_y` 与锁存目标误差不超过 `1e-4 m`。

这里没有 pelvis 到目标的 6 cm 或其他到位门槛。双 ACK 后 active 才释放原 Fixed
HIT；`0.25 s` 超时、拒绝、stale、session/sequence 回退、Controller 重启、
emergency stop、双 HIT 或推理异常均 fail-closed。HIT/POST_DELAY 不被 RL 覆盖；
hitter 进入 OUTWARD/OUTWARD_HOLD 后才显式请求 peer 规则回位。

## 三种模式

- `shadow`：订阅、推理、记录和状态广播；不创建 command publisher。
- `transport-shadow`：要求新鲜 V11 shadow 进程证明，只发布有效且 inactive 的
  `stage`；验证双 ACK，永不发布 HIT/hold/return。
- `active`：完整 Fixed HIT 链路；工作站和机器人入口都要求命令行确认与环境变量
  双门槛。

## 两个现场启动脚本

两个终端按顺序执行：

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
bash scripts/start_yichao_workstation.sh
```

工作站显示 `WORKSTATION BOOTSTRAP READY` 后，在第二个终端执行：

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
bash scripts/start_yichao_robots.sh
```

第二个入口完成一次现场 `YES` 确认并启动 V11 active。新鲜 active attestation
产生后，第一个入口自动把无 command publisher 的 bootstrap shadow 切换成 active，
不再要求现场人员手工停止、查询和重启工作站。看到
`V6R10 WORKSTATION ACTIVE READY` 后再操作 R2。Monitor 地址是
`http://172.16.4.184:8090`。

ROS 固定为 `http://192.168.123.165:11311`，状态 topic 是
`/doubles/yichao_v6r10/status`，Monitor 端口为 8090。Monitor 是只读网页，记录
每条消息并约 10 Hz 建帧索引，支持播放、倍速、拖动与原始 JSONL 下载，无机器人
控制 API。

操作步骤见 [DEPLOY.md](DEPLOY.md)。离线复现：

```bash
.venv/bin/python tools/replay_actor_only.py SOURCE.jsonl \
  --output output/replay.jsonl --summary output/replay-summary.json
```

本仓库实际使用 `../yichao_v3_v9/.venv` 进行导出和测试；工作站 Planner 使用已包含
ONNX Runtime 的 `tabletennis-active-v1` 环境。
