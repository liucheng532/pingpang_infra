# 右侧首拍、未接球原因与重启（2026-09-09）

用户要求右侧机器人固定打第一拍，检查一直不能接球的原因，并明确授权重新启动；用户再次说明机器人有绳索支撑。本次保留工作站输入和两台原生驱动，重启本任务 Planner/Python policy。

## 先前未接球的原因

检查 `continuous_20260909_013859/planner.jsonl*`：01:43:08.865，198 进入 R2 重新校准；01:43:08.877，Planner 永久进入 `FAULT / feedback_invalid:active_policy_stage_not_ready`。此时还没有任何模型决策、命令或 HIT。随后机器人回到 POLICY，但旧机载会话仍标记 invalidated，Planner 也没有恢复。

01:46:40、01:46:49 两次来球分别有 118、132 条有效预测，共 250 条；该日志中模型决策和命令均为 0。因此不能把这轮未接球归因于没有来球。证据为 [trial_diagnosis.json](device_context/right_first_20260909/trial_diagnosis.json)。Web Monitor 当时没有启动；Predictor 正常发布 monitor topic 和球预测，两者不能混为一谈。本次没有启动原 Fixed Monitor。

## 已部署行为

- Runtime 默认首拍为右侧 66，随后 198/66 交替。停球超过 10 秒重新开局仍是 66 首拍；跳过未执行的开局球不改变该归属。
- 在首条命令尚未排队或执行前，R2 重新校准只等待恢复，不永久废弃无事务会话。已有命令之后的 R2 仍不接续旧事务。
- 保留常驻接球、跳过过期球、交接不因 5 秒退出、推理不因 20 ms 退出、HOLD 实测交接，以及后台 16 MiB 分片日志。
- 重启 66 时暴露冻结入口的初始 torso/joint 10 秒超时：新增独立入口的初始输入等待，在真实首条姿态与关节数据收到后才继续冻结启动流程。等待可被用户信号中断；不构造姿态，不发布运动指令，不修改冻结 vendor。

本地及工作站各 76 项首轮测试通过，真实 actor/filter＋冻结 scheduler 的 6 拍合成回放 HIT 顺序为 `66,198,66,198,66,198`，注入交接延迟 6 秒和未到位 6.7 cm 后完成、fault=null。初始等待补丁本地及工作站各 32 项相关测试通过（含模拟 12 秒后恢复、退出信号），两台分别通过无总线 `--check`。这些是软件验证，不等同于物理回球成功。

## 同步与运行证据

源码按逐设备基线比较，备份、明确清单写入及 SHA256 核对；本地设备副本同时更新。工作站首轮 11 文件，两台各 3 文件；初始等待后续工作站/两台各 3 文件。完整差异、备份路径及哈希见 [首轮 sync.json](device_context/right_first_20260909/sync.json) 和 [初始等待 sync.json](device_context/right_first_20260909/initial_wait/sync.json)。没有待同步源码。

传输 session 沿用 `active_20260909_004525_da634c`，新运行子目录 `right_first_20260909_021026`。工作站路径：

```text
/home/odl/codebase/yichao_v3_v9_e14fd5b/output/sessions/active_20260909_004525_da634c/right_first_20260909_021026/
```

旧 FAULT Planner 与两台旧 Python 已退出。198 原生 PID 13850/start 713997、66 原生 PID 3913/start 88262 以及输入 PID 2728928 均保留。两台新 Python 使用右侧首拍/R2 修复；198 首次重启即等待 R2。66 首次启动因没有 torso 输入而退出，已保存 console 后改为持续初始等待并单独重启，记录目录为 `policy_66_initial_wait`。

重启期间实测上游 `/doubles/table_right/torso_pose_origin` 在 4 秒中 0 条，而左侧 1136 条；右侧私有姿态也没有消息。见 [upstream_pose_gap.json](device_context/right_first_20260909/upstream_pose_gap.json)。这是本次启动时的新输入缺失，和先前首拍前 R2 锁死是两件事；尚未证明缺失是遮挡还是动捕端其他原因。已请用户检查右侧标记点与动捕识别。

`yichao_active_inputs:continuous` 常驻等待双机真实 POLICY 就绪，然后自动启动空事务 Planner，输出 `planner_console.log`、`planner.jsonl*`。`readiness.log` 记录启动阶段。不能在该等待阶段声称 Planner 已开始推理或机器人已经能接球。当前指针为工作站 `output/current_continuous.json`；输入原 session 指针保持不变。

观察控制台：工作站执行 `tmux attach -t yichao_v9_active`，分别查看 `policy-66`、`policy-198`。每台出现 `Press R2 to move to the V9 default pose` 后，第一次 R2 校准，完成后第二次 R2 进入 POLICY；双机就绪后 Planner 自动开始，首拍 66。不要在保留会话上重复冷启动两个入口。

02:14 最后核对：[post_restart.json](device_context/right_first_20260909/post_restart.json)。两台原生 PID/start 保持一致，输入仍运行；198 新 Python PID 22253 等待 R2，66 新 Python PID 6546 在 `waiting_initial_inputs` 保持存活并收到 body 数据，超过旧初始化 10 秒仍未退出。`readiness.log` 当前只记录 198 waiting_r2；Planner 尚未启动，`planner.jsonl` 尚未创建。等待66姿态恢复及用户完成双机R2，物理接球尚未复测。

## 02:19 动捕恢复后双机就绪

用户说明66动捕已经恢复。上游右侧4秒530条、私有右侧88条，等待中的66 Python自动继续启动，没有再次重启。用户完成R2后，12秒采样双机状态198 598条、66 457条，最后均为 `policy`、`valid=true`、`sensors_recent=true`、`control_session_invalidated=false`。期间66曾在首拍前多次R2重新校准，最终仍成功回到可用POLICY，没有重复旧的无事务永久失效问题。

`readiness.log` 已记录双机就绪，Planner console 已有 `started=true / mode=active`。新 `planner.jsonl` 已实际写入，最后状态 `IDLE / no_valid_incoming_shot`，`first_hitter=66`、`next_hitter=66`、`fault=null`，日志丢弃/写入错误均0。当前可进行发球测试，本次重启后尚无HIT或物理回球成功证据。此前02:14的无姿态和Planner未启动是历史快照。

证据：[input_recovered_status.json](device_context/right_first_20260909/input_recovered_status.json)、[recovered_process_status.json](device_context/right_first_20260909/recovered_process_status.json)、[recovered_readiness.json](device_context/right_first_20260909/recovered_readiness.json)。

## 02:21 用户报告66摔倒，停止本轮

用户报告右侧66摔倒。立即向确切Planner PID2791337和66 Python PID6546发送SIGINT，后续核对二者退出、command发布者为空；66原生PID3913仍在，198的原生/Python及输入仍保留。没有重启或继续发球。用户随后说明“停了，按了一下R2”，不能据此确认断力矩；本套R2为校准/阶段切换。

Planner文件已核实完整写至停止：`planner.jsonl`、`.0001`、`.0002`、`.0003`、`.0004`共78,210,813字节，elapsed 138.56秒；51543条raw_input、665条relay_step，日志丢弃/写入错误均0，末尾有中断堆栈与summary，`exit_reason=interrupted`。本次有效球预测0、commands_sent=0，没有模型决策、MOVE或HIT；因此日志没有本次未发生的36D/85D推理事件。双机遥测、收到的球预测、等待原因、控制阶段及软件停止记录均保留。不能从“无Planner命令”直接推断摔倒原因，底层policy仍曾持续运行。

停止证据：[right_fall_stop_verified.json](device_context/right_first_20260909/right_fall_stop_verified.json)；日志证据：[fall_log_verification.json](device_context/right_first_20260909/fall_log_verification.json)。工作站运行目录另存 `right_fall_log_summary.json`、`right_fall_software_stop.json`、`right_fall_policy66_console.log`。当前不再处于上一节的可发球测试状态。

## 66向固定位置移动的核查

用户观察66放到桌子中间一些后会向外走。查明66入口仍带 `--bootstrap-outward-hold`，198带 `--startup-home-current`；此前更改的是Planner首拍归属，底层启动角色没有一起切换。66在每次reset时将实测pelvis y写成 `outward_target_y`（同时记录episode_start_x），进入OUTWARD_HOLD。该目标不会随之后人工挪动自动更新，HOLD仍持续将目标减当前位置作为 `target_base` 输入底层policy，不能将HOLD理解成没有电机输出。虽然scheduler默认让位参数为0.9125，当前bootstrap明确以实测位置覆盖它，不能声称66此次启动被硬编码命令去−0.9125m。

只读检查66实际机载recorder：metadata complete，valid_frames=2165，所有实际phase_id=3（OUTWARD_HOLD），记录的target_base_y与outward_target_y−pelvis_y逐帧完全一致。多次reset对应目标约−0.580、−0.543、−0.495、−0.042、−0.151、−0.150m。最后一段目标保持−0.14956m，实际最终−0.45644m，记录误差+0.30688m；目标并没有设为实际到达的外侧位置。较早−0.15116m目标段，末帧pelvis z约0.0987m且姿态明显倾斜，提供摔倒进一步分析线索。这些数据证明存在目标跟踪偏差，不能单凭日志区分自主漂移、人工挪动、绳索作用、坐标/镜像或底层策略原因。

证据：[right_recorder_metadata.json](device_context/right_first_20260909/right_recorder_metadata.json)、[right_hold_target_analysis.json](device_context/right_first_20260909/right_hold_target_analysis.json)。该核查没有修改或重启控制程序。
