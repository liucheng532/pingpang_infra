# Fixed → Yichao 接口对齐与双命令入口

用户已明确选择 `20260906_v3_planner_190_199`，默认 actor 199＋safe_filter_v3，使用交付配套 V9 i19000。现有成功基线 Fixed＋V11 resume42500 保留，不将其底层与 Yichao 混用。

## 2026-09-09 接口修正与当前验证

用户要求集中完成输入输出适配。本轮补齐了“模型每拍一次”与“协议持续轮询”的边界：

- `Relay.poll(record)` 仅用于 PREPARING/CLEARING/COMMITTED/RETURNING 的既有事务推进，不提供 actor/safe 数组或 pipeline，不能借轮询重新计算目标。最新机器人状态、传感器和时钟检查仍执行。
- `RuntimeRelay` 在等待 ACK 时沿用已锁存的目标、sequence 和 token；其他 shot 的预测不再替换本拍上下文。同 shot 的新预测在 HIT 前可以刷新击球信息，TTS 始终按原接收时刻老化，HIT 前保留 0.30 秒有效期检查。
- 只有匹配的应用反馈更新对应机器人“上次实际应用目标”；单机 ACK 不会替另一台确认。
- PREPARED 选择 CLEAR、NEED_RETURN 选择 RETURN 时仍用最新位置历史与双机归一化关节构造安全特征。此处历史存在间断仍会拒绝，未放宽阈值或复用过期 85D。

本地和工作站各 25 项相关测试通过（7 项新增事务边界、5 项冻结 scheduler/relay、13 项特征及输入测试）。完整文件回放使用真实 actor 199、filter 和冻结 V9 scheduler，285 步完成两拍，66/198 各一次 HIT，MOVE 4、CLEAR 2、HIT 2、RETURN 4；合成运动轨迹不代表物理执行或实机击球质量。

已按旧哈希比较远端、备份、同步并核对新 SHA256，更新本地工作站镜像。同步范围是两个工作站 supervisor 模块、新测试和工作站 manifest；这些模块不是机载 CommandReceiver 的依赖，没有适用的机载业务改动。证据见[同步清单](device_context/shot_interface_20260908T161505Z/sync.json)、[工作站测试](device_context/shot_interface_20260908T161505Z/remote_tests.json)、[两拍回放总结](device_context/shot_interface_20260908T161505Z/local_synthetic_relay.summary.json)。

此前两个启动入口已由用户实际运行，并在 `v9_20260908_235934_09016b` 收到有效球、生成目标和双机 shadow 受理，见[现场记录](YICHAO_VALID_BALL_SHADOW_2026-09-09.md)。本轮事务修正及上一轮 TCP_NODELAY 尚待现场复测；不能将合成测试或维度对齐宣布为真实输入时序/坐标验收。active 仍未开放，本轮未启动现场程序。

## 核对结果

| 项目 | 现有 Fixed | Yichao V3 及对齐方式 |
| --- | --- | --- |
| ROS 输入 | 双机 `/doubles/{table_left,table_right}/state`、`/doubles/ball_prediction` | 可复用字段协议；独立机载使用私有 session namespace |
| base 与关节 | state 提供共同 origin 的 pelvis XYZ/四元数、LAB 顺序 q[29]/dq[29]、phase/ready | base XY 与相对球历史组装 actor；双机 q 按指定 URDF 软限位归一化，进入 filter |
| 球 | position、velocity、击球位置/速度、racket normal/velocity、TTS、shot ID、source timestamp | 36D 用相对球历史；85D 最后 7 维用击球位置＋**拍速**＋TTS |
| 规划网络输入 | 无 36D learned actor；规则与 relay 使用结构化输入 | 36D actor：30D 双机相对球历史＋4D 当前 base XY＋2D 上拍应用目标 |
| 安全过滤 | 当前 Fixed admission 明确 `cbf_active=false`、`safety_active=false` | 85D 状态与独立 2D 候选输入；集成风险＋候选搜索＋0.45m 解析间距 |
| 目标来源 | home/outward 由机载及 runtime motion config 给定 | actor 输出两台绝对 Y 目标，裁剪归一化动作后乘 1.2m，不能当位移/速度 |
| HOLD | fixed hold 为 no-op；即使消息带 desired_base_position 也不执行移动 | MOVE/CLEAR 使用实际 `set_external_base_target` 路径，不把动作填进 hold |
| HIT | 当前成功 V11 bridge 要求 `prediction_source_timestamp_s`，返回 `PlannerHitUpdate`，再由 agent 处理 teacher/HIT 路径 | 配套 V9 bridge/独立接收器调用冻结 scheduler 的 HIT 钩子；不拿 V9 接收器冒充 V11 接收器 |
| RETURN | Fixed 外围持续发送 return，目标来自配置 | 独立 relay 使用带 session/sequence/shot/token 的明确请求，接收与到位分别记录 |
| 反馈记忆 | Fixed 依靠 phase/token 等推进 | 上拍目标记忆由本任务专用受理反馈更新，不能把 actor nominal 或通用 last_applied_sequence 当 ACK |
| 频率 | 工作站 tick 50Hz | 通信仍 50Hz；actor 每拍一次，双机目标锁存，不能每帧重新规划一拍 |

训练 L 槽位→66/table_right（现场右侧、左手镜像），训练 R 槽位→198/table_left（现场左侧、右手 canonical）。槽位不能按 ROS 字符串 left/right 对接。

训练源码 `_prime_histories` 的球历史由位置/速度/加速度按 `_step_dt()` 回推；base 历史来自仿真状态记录。当前真实适配仍将 20ms 接收时间网格标为诊断假设，真实源时钟、共同坐标/朝向和历史初始化不能由消息格式校验代替。

依据：本地工作站副本的 `scripts/run_v9_real_fixed_relay.py`、`doubles_planner/fixed_relay.py`、`real_ros_runtime.py`；两台对应 V11 bridge/agent；Yichao 交付 schema、DEPLOYMENT 和 `doubles_planner/v0.py`。未修改任何现有 Fixed/V11 文件。

## 已实现的两个入口

两个终端都在工作站独立目录 `/home/odl/codebase/yichao_v3_v9_e14fd5b`。不需要手动激活 conda，入口会选择本任务 `.venv`。

终端 1：

```bash
python3 -B scripts/run_yichao_stack.py
```

终端 2：

```bash
bash scripts/start_yichao_dual_robots.sh start shadow
```

**当前这两条是限时机载 shadow 入口，不是 active 接球入口。** `start active` 在任何组件启动前明确拒绝；不以假命令或关闭门禁宣称交付真机控制。

终端 1 检查现有控制发布者，选择已有完整 Predictor 输入或启动本任务私有 Predictor，生成唯一 session，并等待终端 2；终端 2 先只读核查两台 SSH 和控制进程，通知终端 1 开始私有姿态转发，再启动双机被动遥测/V9 shadow。两台私有 state 出现后，终端 1 运行 actor/filter/relay 并记录，姿态源保持到机器人退出。无需手填节点名、session 或多条 SSH 命令。SSH 复用现有双机启动器的 `/home/odl/.ssh/pingpang` 和有线管理入口；不修改认证、网络或共享环境。

日志在 `output/sessions/<session>/`：各进程 stdout/stderr、姿态与 relay JSONL、工作站/双机退出结果。私有 Predictor 另有独立运行目录和原始动捕源证据。退出只处理入口创建的确切子进程；机载遥测和 policy 各自还有固定时限。没有“停止他人程序”选项。

原方案仍有控制/状态发布者时，不自动启动 Predictor 去接入其输入链，也不启动本任务机载 policy。用户此前要求保留现有方案；实机运行仍需明确交接。

## 验证与剩余工作

已完成本地 4 项启动生命周期测试、Shell 语法检查；工作站验证记录为 `doc/yichao_v3_v9/two_terminal_workstation_checks.json`。源码明确备份、同步、SHA256 验证及本地设备副本更新见 `doc/device_context/two_terminal_1788880869638632924/`。

同时修正私有 Predictor 启动器的信号退出：收到信号立即进入限时关闭流程，而非继续等原运行时限导致 SDK 子进程遗留。该修正未修改 Predictor 原始源码。

双命令组合尚未完成现场端到端复测；真实来球/源龄/坐标及专用受理反馈未验收，不能宣布 active 可运行。此前两次单独协调的机载 shadow 已通过启动运行，但不等于本次新启动器完整验证。24拍 sim2sim 的4/24击球质量结论保留。

## 23:25 交接与遥控器核对

用户已明确两台交给 Yichao 测试。只读进程检查：198 无 g1_control/policy；66 仍运行 V11 g1_control PID 6712 和 policy PID 6870。本轮未停止它们，等待现场确认66是否仍依赖原控制保持姿态。后续操作重新核对确切 PID。

配套 V9 `deployment_runner.py:75` 的 active 校准先等待 R2，再缓慢进入 nominal，随后等待第二次 R2 开始控制；运行中 R2 调用 `calibrate(wait=False)`，会重新运动到校准姿态，不等价于断力矩急停。当前 shadow 跳过校准和电机输出，不需要按 R2。原手册关于再次 R2 退出的简写不能覆盖该源码行为。
