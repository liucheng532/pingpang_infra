# Yichao actor 199＋V9 i19000 首次 active 试运行

用户本轮明确要求上真机、聚焦核心功能，并说明现场安全、随时可以测试。本轮按该授权推进，不再要求重复确认绳索和急停信息。

## 实现与验证

已接通独立 active 双终端入口：actor 199＋safe_filter_v3 → MOVE/CLEAR/HIT/RETURN → 配套冻结 V9 i19000 scheduler、镜像及 ONNX → 私有 g1_control。默认 shadow 入口保留；active 需显式选择。工作站等待两台各连续两条新鲜 POLICY 状态后，才开始 Planner。校准前禁止 Planner 动作受理，运行后重新校准使旧事务失效。仍保留源龄、坐标/历史输入检查、事务超时和真实位置/速度完成反馈。

软件实现和实际验收分开记录：`experimental_active=true`，`real_input_accepted=false`、`physical_accepted=false`；没有把旧 shadow 的受理记录改成物理完成。真实 actor/filter＋冻结 V9 scheduler 的 active 模式文件回放完成两拍、双机各一次 HIT；位置来自合成轨迹，此结果不是实机运动证明。

- 本地 158 项测试通过（154 项与 4 项启动测试分进程执行；离线审计钩子禁止子进程，不能在同一进程内运行启动测试）。
- 工作站 48 项相关测试通过。首次与预检并发时有一项测试争用启动器锁，顺序重跑 48 项通过，没有绕过运行锁。
- 工作站调用两台机载 active `--check` 通过：源码/环境清单、独立 g1_control 和库、V9 ONNX ABI、动作库校验。该检查没有打开硬件总线。
- 工作站 18 个文件、两台各 7 个文件同步完成，替换前比对各设备基线、备份，替换后逐文件 SHA256 一致，设备本地副本已更新。原 Fixed/V11 未修改。

同步、测试和启动记录：[device_context/active_release_20260909](device_context/active_release_20260909/)。原生控制资产来源：[active_control_assets_20260909/README.md](device_context/active_control_assets_20260909/README.md)。本地测试日志为 `yichao_v3_v9/output/active_release_local_tests.log` 和 `active_release_terminal_tests.log`，回放为 `active_release_replay.jsonl.summary.json`。

## 两个启动命令

两个终端均在工作站 `odl@172.16.3.126`，目录 `/home/odl/codebase/yichao_v3_v9_e14fd5b`：

```bash
# 终端 1
python3 -B scripts/run_yichao_stack.py start active

# 终端 2（终端 1 显示 ready 后）
bash scripts/start_yichao_dual_robots.sh start active
```

本轮已由 Agent 启动输入端，不要重复启动。工作站输入会话为 `yichao_active_inputs`；机载控制窗口由工作站 tmux `yichao_v9_active` 管理。

每台先按一次 R2 到 V9 默认姿态，校准完成后再按一次 R2 进入 policy。必须等输入端显示双机 POLICY 和开始 Planner 后才发球。本轮目标为两次完整交接；完成或 Planner 退出后，机载控制程序继续运行。

原生 g1_control 一启动就有约 3 秒零位过渡，早于 R2；R2 不是断力矩急停。Ctrl-C 只退出软件，不代表电机断力矩。结束测试、现场已解除电机输出后，可执行 `bash scripts/start_yichao_dual_robots.sh stop active --motors-disabled` 清理本会话软件。

## 本轮现场进度

输入会话 `active_20260909_004525_da634c` 已启动 Predictor、私有 torso relay 和双机 readiness 观察。双机 g1_control 和非 shadow V9 policy 已真实启动，四个 tmux 窗口均存活，两台均报 `waiting_r2`。66 启用左手镜像、初始 OUTWARD_HOLD；198 canonical、初始 HOME_HOLD。已通知用户两次 R2 操作，尚未收到 POLICY、来球或 HIT/交接结果。原生驱动已启动，不能再称本轮没有电机程序；不以进程启动代替物理动作验收。


## 日志上限故障及恢复（同日后续）

首次启动在等待 R2 期间，工作站 `poses.jsonl` 达到诊断 Writer 的 16 MiB 上限，抛异常并触发工作站子进程清理，造成两台 torso 输入中断。198 完成默认姿态校准后收到了第二次 R2，在首次 policy 输出前因 torso 接收龄 22.298 秒退出；66 在校准开始时因 torso 接收龄 46.500 秒退出。原生 g1_control 均持续运行，未停止或重新启动。本轮没有 Planner relay 日志、没有真实来球到 Planner 或 HIT/交接结果。

用户明确要求像 Fixed 一样常驻，不因诊断条件频繁退出。已作以下修复并在本地、工作站各通过 32 项测试：

- 持续姿态输入及 active relay 日志按每段 16 MiB 自动创建新分片，原始证据保留，不因大小限额退出。
- 机载校准/输出遇到源龄不足时进入 `waiting_inputs`，进程保持运行并发布真实状态，输入恢复后丢弃等待前算出的动作，下一循环用新状态重新计算。POLICY 阶段发生输入中断仍使旧 Planner 事务失效，不伪造 ACK 或物理完成。
- 增加启动失败恢复工具：只接受本会话两个原生驱动存活、两个 Python policy 已退出、Planner 尚未开始的状态；校验原生 PID/starttime、二进制和私有 LCM，仅重建输入及退出的 policy，不重启底层驱动。

工作站 9 文件、机载各 3 文件已备份同步并核对 SHA256，见 [恢复同步与测试](device_context/active_logging_recovery_20260909/)。同会话恢复目录为 `recovery_20260909_005802`，实时状态已确认双机重新处于 `waiting_r2`，五个运行窗口全部存活，已通知用户重新进行两次 R2 操作。

后续另将 `wait_active_ready.py` 默认等待改为无超时，保留可选 `--timeout-seconds`，2 项就绪测试通过，工作站已同步并验哈希，见 [等待行为同步](device_context/active_ready_wait_20260909/)。本次恢复时已启动的 readiness 进程加载的是之前 600 秒等待实现，磁盘更新不会热替换该进程；此更新适用于后续启动。


## 真实 active 两次 HIT、第一次交接及用户 R2 暂停

恢复后两台都完成 R2 校准并进入 POLICY。本次真实输入中共 233 条有效球预测，属于 mocap-1、mocap-2 两个 shot；发送16条协议消息，其中6条重发、10个唯一命令。198 首拍 HIT（motion 182），随后双机 RETURN 完成第一次交接；66 第二拍 HIT（motion 86），随后等待让位完成时触发 `handoff_timeout`。击球动作执行不等于已确认触球或回球质量。

Planner 最后记录66 Y=−0.846319m、目标−0.912500m，误差约6.62cm；198让位完成。冻结V9代码可因移动参考结束进入OUTWARD_HOLD，独立完成反馈仍要求位置误差≤6cm，所以不能用OUTWARD_HOLD冒充已精确到位。Planner退出后，双机policy、g1及工作站输入继续运行；随后姿态日志已有三个分片，证明超过原16MiB限额没有再切断输入。

用户随后说明“R2退出，请检查状态”。工作站时间2026-09-09 01:10:43只读核对：66和198均重新校准后进入 `waiting_policy_r2`，不是进程退出或断力矩。66原生PID6866/Python7518，198原生PID13850/Python15094（仅此时快照）；同一原生驱动自首次启动一直保留。工作站输入恢复进程PID2728928仍在，Planner已结束。没有在用户R2之后启动Planner续接，也没有再次启动/停止机载程序。该R2使旧控制代次失效，先前计划的只恢复旧Planner事务不再适用于当前状态。

超时常驻、OUTWARD_HOLD按实际稳定间距释放、连续接球与日志续接的补丁目前仍在本地验证，尚未同步设备或用于真实发布。下一轮须重新建立有效会话，不能把这些本地改动当作设备当前行为。


## 连续接球补丁部署及66换电后恢复

用户要求超时不再中断接球、减少中断性限制、记录Planner分析日志；随后说明66没电后换电重启。本轮修复已同步三设备并验SHA256，本地/工作站各106相关测试通过，两台无总线加载检查通过。真实actor/filter＋冻结scheduler文件回放注入6秒延迟和6.7cm未到位，双端均连续完成6拍，各机3次HIT。此前“补丁仅本地未同步”已是历史。

恢复保持198原生PID13850/starttime713997及工作站输入PID2728928；退出旧198 Python15094，启动换电后的66原生PID3913，启动两台更新后的Python policy。两台均到waiting_r2，工作站 `yichao_active_inputs:continuous` 常驻等待双机POLICY，再从空事务启动新Planner。没有恢复此前mocap-2的HIT事务。当前实时状态、软件语义和日志关联详见 [连续接球记录](YICHAO_CONTINUOUS_PLANNER_2026-09-09.md)。


上述换电恢复随后已完成两台第二次R2，双机进入POLICY、常驻Planner实际启动。新 `planner.jsonl` 已轮转到`.0004`且持续更新，Planner fault=null、日志丢弃/写入错误均0，目前等待有效来球。本轮最新软件与现场状态以连续接球记录为准，不再沿用“等R2/Planner尚未启动”。
