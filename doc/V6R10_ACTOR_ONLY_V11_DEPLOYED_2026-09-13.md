# V6R10 actor-only + V11 i42500 独立交付记录

日期：2026-09-13

> 最终软件就绪更新：198 有线链路已恢复，双 V11 Controller shadow、实时进程证明及
> transport-shadow 双 ACK 均通过；sequence 8704 双侧约 20 ms 完成，恰好两个 stage、
> 零 HIT，Controller 全程 `shadow=True`。已增加与 V3+V9 相同用法的
> `start_yichao_workstation.sh` 和 `start_yichao_robots.sh`；前者自动完成无命令 torso
> bootstrap，后者启动 V11 active 后，前者凭新鲜 active attestation 自动切 active。
> 本地 44 项、工作站 42 项通过（2 项无 Chrome 跳过），41 文件 manifest SHA256
> `f27480032a99ee3f135626755ebf377f85752ed9ed6584efef03a19c4d370cee`。bootstrap
> 实机 ROS 演练双 torso 在线、双 command publisher 为空并干净退出。软件具备进入
> active 测试条件，但尚未把 R2、站稳、击球或回球表述为物理通过。

> 续查更新：dead tmux 已清理，V6R10 fail-closed、首次 shot 接收时刻、attestation
> 失败清理、严格 actor sidecar/ONNX 合同和双 torso Monitor 已补强；本地 41 项、
> 工作站 39 项通过（2 项无 Chrome 跳过），新 manifest SHA256 为
> `ebf513e30b55ccba96fda32dc917444b2b396c4a18c026e667141368b886dea4`。
> 当前 shadow `v6r10_20260913_152240_8fc91990dc` 已实收双 torso 约 294 Hz，双
> command publisher 为空。剩余 blocker 是工作站到 198 的 USB 有线接口
> `enxec1ac30040d6` 为 `carrier=0`，详见
> `doc/device_context/v6r10_v11_readiness_20260913/README.md`；Controller shadow、
> transport-shadow、active 和物理验收仍未完成。

## 结论

已在旧 `v9_runtime.YichaoRuntime` 之外完成独立的 V6R10 部署实现，并同步到工作站：

```text
/home/odl/codebase/yichao_v6r10_v11_adapter
```

本交付没有修改或覆盖旧 V3+V9、Fixed、Predictor、V11 Controller、共享标定、
ROS 11312 或旧 8089 Monitor。它不取代当前 V9 生产部署，只增加一条明确隔离的
V6R10+V11 路径。

生产路径只加载 `model630_actor.onnx` 及 sidecar。frozen learned barrier、51×51 搜索、
risk 阈值和 ensemble 风险头均未导出、未加载、未旁路计算；状态与 Monitor 固定显示
`filter_mode=off`，barrier/risk 为 `not_evaluated`。

## 实现范围

- 联合 actor 由 `model_630.pt` 重建并导出动态 batch ONNX，网络为
  `36 -> 256 -> 256 -> 2`、ELU，使用冻结 mean/std，输出确定性 actor mean。
- 显式身份固定为 `left -> table_left -> 198 -> action[0]`、
  `right -> table_right -> 66 -> action[1]`，不借用 V9 `ROBOT_ORDER` 推断模型下标。
- 36D 使用 5 帧左右相对球历史、左右当前 XY 和上一组完整双 ACK 目标；base Y 除数
  为 1.0，previous target 除以 0.9，初始值为 `[+0.35,-0.35]`。
- 87D audit vector 单独保留并记录，但不输入 actor，也不是 barrier 依赖。
- 首球 interval 为 1.65 秒；后续仅在不同 shot 完整双 ACK 后，按接收时刻差更新，
  裁剪到 `[1.5,1.8]` 后使用 `(value-0.8)/0.8`。
- actor 输出先裁剪到 `[-1,1]`，再映射到 `[-0.9,+0.9]m`。hard guard 保留
  Fixed 选定 hitter 的 actor target，只向外投影 peer，使 `left-right >= 0.50m`；
  无法投影、非有限或维度异常时永久禁用该 shot，并请求规则 home。
- 原 `FixedRelayPlanner` 继续决定 hitter、reservation、commit token、HIT 和回位。
  首次 commit 先锁存双机 actor pair 并发送 `stage`；同 shot/pair 两侧命令 ACK 后才
  释放 HIT。ACK 只核对 session、sequence、transport error 和 `target_base_y`，没有
  pelvis 距离或 6 cm 到位门槛。
- previous target 仅在双 ACK 后原子更新；单侧 ACK、无效球、fallback 和失败 shot
  不更新。0.25 秒超时、state/ball stale、Controller 回退/重启、双 HIT 等均关闭该球。
- HIT/POST_DELAY/OUTWARD/OUTWARD_HOLD 由 Fixed/V11 持有；peer 只在观察到 hitter
  进入 OUTWARD/OUTWARD_HOLD 后显式回 home。
- 三种模式分别为无 command publisher 的 `shadow`、只发布 inactive `stage` 且永不
  HIT 的 `transport-shadow`、以及需要双显式现场确认门槛的 `active`。
- 工作站固定 ROS `http://192.168.123.165:11311`、状态 topic
  `/doubles/yichao_v6r10/status`、Monitor 8090。启动前拒绝已有 stack 进程、已有双
  command publisher、ROS Master 不可达或 8090 占用。
- 非 shadow 启动前重新生成 V11 live attestation，核对 launcher、双机 boot/process
  身份、student/checkpoint/sidecar、HIT/move teacher、动作 manifest、bridge、scheduler、
  mirror 和 native binary，并固定 residual off、scale 1.0、family v11-teacher。

## 模型与离线验证

| 项目 | 结果 |
|---|---:|
| checkpoint SHA256 | `81501373c08f6f4aa6bae3fdaccdd073939e5346d2b25a8e419ab35c6657ed0c` |
| handoff schema SHA256 | `f2bf9c51e37f675cdb6e063cab325c0bf0eb569bb46c0fb104c075c3e6ed7cee` |
| actor ONNX SHA256 | `cd34836bac0ead105ffe3c9e6320c4267f21e72abd87f1711758e7b16d4e6787` |
| PT/ONNX 最大绝对误差 | `2.384185791015625e-06`，低于 `2e-5` |
| 本地新部署测试 | 35 passed |
| 工作站新部署测试 | 33 passed, 2 skipped（无 Chrome） |
| manifest | 38 文件、5,680,404 bytes，SHA256 `14451b85b8811f07673b8656446d391fcd400b3c576a042435b098f3c4af85a9` |

测试所需的 checkpoint/schema、joint-normalization 和 replay input 作为不可变 audit
fixture 放在 `tests/fixtures`；生产模块不引用该目录，生产 inference 仍只依赖 ONNX。
本地 Chrome 的 1280×900 和 390×844 两种布局、录制、索引回放和无控制 API 均通过。

离线回放使用 169 行 handoff fixture，共 2 个不同 shot：2/2 决策有效，2 次 hard-guard
介入，command 发布数为 0，未导入 ROS。5000 次完整 36D/87D 构造 + ONNX + guard
基准的 p95 为 `0.2611392 ms`，最大 `0.828493 ms`，通过 `<20 ms` 门槛。结果保存在
`yichao_v6r10_v11/output/offline_replay_20260913/`。

旧 `yichao_v3_v9/tests` 在当前未改旧树上实际为 67 passed、3 failed、882 warnings；
3 个失败是既有 RL Monitor HTML 已更新而旧测试仍断言旧 DOM id 的不一致。本交付未为
凑测试数修改旧 V3+V9 文件，因此不能把当前旧套件写成 70 passed。

## 工作站 shadow 结果

最终 shadow 会话：

```text
v6r10_20260913_025308_0e5f054dbc
```

Predictor、Planner、Monitor 的 PID/starttime 均与 owner 记录匹配。Planner 状态可被
Monitor 稳定接收，约 4.7 Hz，内容为：

```text
mode=shadow
planner_identity=v6r10-model630-v11-i42500
filter_mode=off
barrier_risk=not_evaluated
reason=waiting_inputs
table_left.command_published=false
table_right.command_published=false
```

原 Predictor 约 293 Hz 发布无效球；本轮没有 Controller state，因此 Planner 保持
`waiting_inputs`，没有伪造有效球或 actor 决策。Monitor 录制
`rec_20260913_025309_b52575db`，54.373 秒、16,016 条消息、541 帧，
`dropped=0`、`error=null`，并明确 `robot_control_api=false`。

会话随后通过其 owner 入口停止，Predictor/Planner/Monitor 退出码均为 0，
`failed_component=null`。最终 8090 空闲，V6R10 status publisher 消失，左右 command
publisher 均为空；原 8089 Monitor/PID 231463 保留。

启动时发现 Monitor 可能先于 Planner 完成 ROS 注册，造成已注册但未建立 status TCPROS
连接。supervisor 已修正为等待 Planner status publisher 出现后再启动 Monitor，第二轮
shadow 证实 Monitor 同时收到 `ball` 和 `planner`。

## Controller shadow / transport-shadow 未执行

只运行了原 V11 委托入口的 `check shadow`，返回码 1：

```text
Preflight FAILED: tmux session already exists: v11_dual_robots
```

只读展开显示该遗留会话的四个 pane 均 `dead=1`，但原入口按设计仍拒绝越过。另有：

- 工作站只有 `192.168.123.165` 接口在线，对应 66 的 `192.168.123.164` 可达；
  198 所需接口为 DOWN，`192.168.124.164` 不可达。
- 双 torso publisher 和 ball publisher 在停止 workstation shadow 后均为空。

本轮没有擅自清理遗留 tmux、启用网卡或绕过 torso 门槛，因此没有启动 V11 Controller
shadow、没有生成 live attestation，也没有进行 transport-shadow 双 ACK。工作站和双机
均未进入 active，未按 R2，未发 HIT，未做站稳、击球或回球物理验收。

## 当前入口

操作顺序与安全限制见 `yichao_v6r10_v11/DEPLOY.md`。现场条件恢复后仍应先执行：

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
./scripts/start_v11_robots.sh check shadow
```

只有该检查通过，才可以由用户决定是否继续 Controller shadow 和 transport-shadow。
`active` 必须等待用户重新确认摆位、绳索、人员、急停和双机状态；本记录不构成该确认。
