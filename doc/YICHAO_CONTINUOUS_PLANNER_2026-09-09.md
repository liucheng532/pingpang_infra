# Yichao 连续接球与 Planner 日志（2026-09-09）

用户要求继续测试，减少导致中断的额外限制，超时后继续接球，并保存 Planner 日志供后续分析。本次沿用 actor 199＋safe_filter_v3＋配套 V9 i19000，修改独立 Yichao 部署；原 Fixed/V11 未修改。

## 本次行为

- active 默认 `--seconds 0 --shots 0`，持续运行；无球时等待，不在两拍或固定运行时长后结束。
- active 取消 5 秒交接退出和 20 ms 推理超时退出。慢计算保留耗时记录。
- 准备、CLEAR 或球预测过期时，仅结束错过的该拍，继续等待下一拍；不能对已经过去的来球补发 HIT。
- 已发送但 ACK 尚未收到的 HIT 只重传相同 sequence/token/原始有效期。机载可返回缓存 ACK，过期的新执行仍拒绝；不会为了重试再生成一次 HIT。明确未执行的 RETURN 可换新 sequence/token 重试。
- 暂缺状态、时钟、短暂输入间断或历史间断改为等待恢复，不把这些条件锁成整个运行的故障。机载 `waiting_inputs` 恢复不再永久废弃控制会话，等待前算出的动作仍丢弃。
- 冻结 V9 可以在参考动作结束时进入 HOLD，即使距目标尚有误差。交接可依据双机命令已受理、实际位置分居两侧、间距至少 0.45 m、横向速度不超过 0.15 m/s、HOLD 稳定至少 0.1 秒继续。日志单独记录此放行及实际位置/目标；不会改写成精确目标到位。RETURN 在参考结束后的同类稳定状态可允许下一拍 MOVE。
- R2 重新校准仍是用户主动改变控制状态，旧事务不自动恢复。本次换电后的重启建立全新 Planner 事务，不从上次超时日志恢复 HIT。

这些改动消除了上述软件中断原因；不把持续运行等同于每球都能物理接住。缺少有效来球、机器人断电或尚在 R2 等待时没有可执行的接球条件。

## 日志

`tools/run_runtime_relay.py` 每次调用自动创建独立日志路径，也支持显式 `--output`。持续 JSONL 按每段 16 MiB 追加分片（`.0001`、`.0002`……），不覆盖旧记录。后台写入与控制循环分开；日志队列丢记录或磁盘写入失败均计数，不因日志大小或诊断写入错误终止控制。

| kind | 用途 |
| --- | --- |
| `run_start` | 会话、进程、actor、运行参数、源码 SHA256、开始时间 |
| `raw_input` | 原始球预测/双机状态，保留源时间、接收时间和发布者 |
| `planner_input` | 每拍实际 36D actor 输入、85D filter 输入、历史/关节/上一目标依据 |
| `planner_decision` | raw nominal、过滤/投影目标、候选风险分数、阈值、击球机器人 |
| `relay_step` | MOVE/CLEAR/HIT/RETURN 状态、耗时、等待或跳拍原因、完成拍次 |
| `command_tx` / `command_feedback` | 完整命令身份、重发标志、受理/完成/拒绝反馈 |
| `command_not_sent` | 发布前已经过期且尚未发送的命令 |
| `handoff_release` | 参考动作 HOLD 后按实测位置继续交接，精确完成证据单独保留 |
| `slow_planner_step` | 超过 20 ms 的步骤，不再因此退出 |
| `clock_probe` / `clock_reply` | 双机时钟交换和传输时间 |
| `summary` / `run_exception` | 退出原因、堆栈、计数、日志丢弃/写入错误 |

日志事件带工作站 monotonic/wall 时间；每拍通过 session/shot，命令通过 robot/sequence/token 关联。完整原始记录用于后续重算，不能只看 console 尾部。

## 验证与同步

本地 106 项相关测试通过。工作站首次测试遇到现有输入进程持有的启动锁，已将该测试锁隔离到临时目录，再跑 106 项全部通过；没有停掉输入进程或取消正式入口锁。两台机载 `run_onboard_active.py --check` 均通过，不打开控制总线。

本地真实 actor/filter＋冻结 V9 scheduler 的文件回放，注入每次让位 6 秒延迟和 6.7 cm 未到位，完成 6 拍/6 次交接，198、66 各 3 次 HIT，最终 COMPLETE、fault=null。这是合成位姿/阶段故障注入，并非实机回球质量证明。工作站同组回放结果见同步目录。

源码首轮工作站 17 文件、机载各 5 文件同步，后续补同步测试锁隔离及清单。均逐文件比对各设备基线，备份后写入并核对 SHA256，更新本地设备副本。工作站环境依赖的 `requirements.lock` 本就与笔记本不同；资产清单已绑定工作站原有文件 SHA256，未修改/安装工作站依赖。

证据目录：[device_context/continuous_planner_20260909](device_context/continuous_planner_20260909/)。`initial_plan.json`/`initial_sync.json` 为第一轮源码，`plan.json`/`sync.json` 为后续测试文件，`workstation_followup_sync.json` 记录工作站专属资产清单和本次恢复脚本。运行源码与机器人部署清单同步后不会自动热替换已运行的 Python。

## 换电及当前启动

用户说明 66 刚没电，已换新电池重启。只读确认 66 原 policy/g1 已不在；198 原生驱动和旧 Python 仍在，最后控制阶段 waiting_policy_r2。工作站 Predictor/姿态输入保持运行。

本次恢复脚本 `restart_after_battery.py` 保留 198 原生驱动和工作站输入，结束本任务旧的 198 Python，启动换电后的 66 原生驱动，再为两台启动更新后的 Python policy；Planner 从空事务开始。实际启动结果和日志路径在下文补记。


恢复实际完成，工作站启动脚本退出码0，两台新policy均waiting_r2。198原生PID13850/starttime713997保持不变；66新原生PID3913/starttime88262。Planner等待窗口已创建：`yichao_active_inputs:continuous`；输入窗口仍为`:stack`。已通知用户每台第一次R2校准、第二次R2进入policy，等Planner started后发球。此次软件更新的真实连续击球尚未复测。

本轮具体目录：

- 工作站：`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/sessions/active_20260909_004525_da634c/continuous_20260909_013859/`。`restart.json`、旧窗口快照、`readiness.log` 已创建；双机POLICY后创建 `planner.jsonl` 与后续分片，console为 `planner_console.log`。
- 198：`/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z/output/active_20260909_004525_da634c/continuous_20260909_013859/policy_198/session_20260909_013935_692126/`。
- 66：`/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z/output/active_20260909_004525_da634c/continuous_20260909_013859/policy_66/session_20260909_013935_476719/`。
- 继续保留的输入：原恢复目录 `recovery_20260909_005802/poses.jsonl*`、Predictor日志。新旧Planner共用此输入源，按工作站时间关联；不能把新的恢复子目录当成另起一套Predictor。

工作站6拍回放也已通过（`workstation_six_shots_final.log`）。正式启动前只读检查最初在注销ROS订阅后计算新鲜度，引入了清理耗时，误报无新鲜状态；已把采样时刻放回清理前再检查，确认198实际是等待R2，没有因此停止任何程序。工作站资产清单中仅有原有依赖文件哈希与本地不同，已绑定其实际原有 `requirements.lock`，随后完整清单及回放通过，没有绕过冻结资产检查。


最新只读状态已推进：双机完成第一次R2校准，均 `waiting_policy_r2`、`calibration_generation=1`、`control_session_invalidated=false`，身体/IMU/torso输入新鲜；已通知用户各再按一次R2进入policy。此时尚未创建 `planner.jsonl`，因为readiness仍在等第二次R2；启动与等待日志已写入。最终逐文件核对工作站19项、机载各5项，均与各自本地设备副本一致，见 `final_consistency.json`。


随后双机均完成第二次R2进入POLICY，readiness实际通过，`yichao_active_inputs:continuous` 已运行常驻active Planner。`planner.jsonl` 已创建且超过首个16MiB分片，日志记录 `continuous=true`、`fault=null`、`log_dropped_records=0`、`log_write_errors=0`；等待有效来球。已告知用户可以发球。此进展取代上一段“等待第二次R2、尚未创建Planner日志”的时点状态，见 `planner_start_status.json` 及后续 `live_planner_final.json`。


最后一次读取覆盖全部5个Planner日志分片（首文件＋`.0001`至`.0004`），总计约71.6MB。最新relay_step距离读取约77ms：`state=IDLE`、`reason=no_valid_incoming_shot`、`fault=null`、`continuous=true`、日志丢弃0、写入错误0；尚无本轮有效来球决策/HIT。已实际证明在本轮长于原日志限额和旧超时的等待中保持运行，未将无球等待冒充物理连续击球验收。
