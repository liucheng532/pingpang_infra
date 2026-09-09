# 66首颗来球前保持校准姿态（2026-09-09）

> 21:58 撤回说明：用户实测本方案不运行动态V9策略、66站不稳。已确认设计错误，固定关节PD不能替代动态平衡；“暂停推理的等待”标为policy也不准确。本文以下实现已从磁盘代码撤回，不再作为可用待命方案。当前默认恢复原版动态V9，另新增显式HOME_HOLD启动参考候选，尚未物理验证。三设备代码已同步，但旧机载进程没有热加载；Planner已停止，输入/双机Python/原生仍在。见[最新方案、验证和进程状态](YICHAO_DYNAMIC_HOME_CANDIDATE_2026-09-09.md)。

> 21:42 运行更新：用户明确要求启动。恢复后的Yichao＋V9已运行，新会话`active_20260909_213838_d7b03a`（工作站时钟）；双机原生和Python均已核对存活，最新均waiting_r2、HOME_HOLD，66保留镜像。工作站输入已运行，Planner等待双机完成两次R2后启动；此时尚无本轮MOVE/HIT。实际default到位、站立及首球切换仍未确认。输入日志位于工作站独立目录`output/sessions/active_20260909_213838_d7b03a/`，Planner启动后写入该目录`relay.jsonl`。本段取代下文21:36全停状态，勿重复启动。见[启动证据](device_context/restored_idle_start_20260909_213930/started.json)和[双机进程核对](device_context/restored_idle_start_20260909_213930/robot_processes.json)。

> 21:36 最新状态：用户报告回退版按R2后66仍移动，明确要求撤销回退、改回首球待命代码。本文所述首球固定姿态等待、66当前位置HOME_HOLD及最终校准目标补发均已恢复；本地47测试通过，工作站5文件、机载各3文件备份同步并核对SHA256，本地设备副本一致。当前程序已按此前kill要求全部停止，本次未重启，实机待命及站立尚未复测。见[恢复同步清单](device_context/restore_first_ball_idle_20260909/sync.json)、[验证结果](device_context/restore_first_ball_idle_20260909/validation.json)和[停机核对](device_context/yichao_stop_20260909_213037/robots_final.json)。

20:55曾按用户当时要求撤回上述改动；该回退现已被21:36恢复取代。历史证据：[回退清单](device_context/revert_first_ball_idle_20260909/sync.json)。本次恢复使用回退前备份，入口SHA256为`1503e79e83b35c71e6e6fd9594f58994f4dc53a036956383735b6a68b63b608b`。仅恢复已实现的首球待命逻辑，没有将移动/不稳原因认定为已解决。以下保留此前实现及现场操作记录，运行状态以本段最新说明为准。

用户最终明确：第一颗球还没来的时候66不要自己动；球来之后首拍允许正常横移，后续击球和交接都正常。本次只修改独立Yichao入口的启动待命，不限制Planner首拍目标，也不禁用66后续动作。

## 19:20 状态核对与校准目标补充修复

用户明确本轮排查的是此前 Yichao＋V9 的首球前乱动；当前 Fixed＋V11 由他人调试，不能接管。本次只读核查三台 SSH、进程、启动参数、tmux 控制台和 ROS 节点存活情况：工作站运行 Fixed relay PID3479736；66 为 V11 policy PID3307 / g1 PID3152，198 为 V11 policy PID8084 / g1 PID7974，模型均为 resume42500。采样时两台控制台最后停在首次 R2 校准提示，不能仅凭 HOME_HOLD/OUTWARD_HOLD 日志断言已进入 policy。没有本任务 Yichao 控制进程；ROS 中旧 Yichao 198 注册的 API 连接拒绝，属于失效注册，本轮未清理。证据：[完整状态快照](device_context/first_ball_idle_20260909/current_status_20260909_192051.json)。

三设备此前首球待命补丁哈希均与上次同步一致。但上次重启止于 waiting_r2，未走到补丁后的首球等待，故仍未完成物理验收。

本地核查另复现一个确定的校准边界遗漏：当实测关节与 nominal 的最大误差不超过 0.01 rad 时，校准循环会完全跳过指令发布，实测接近目标不代表原生驱动最后收到的目标也正确。新增回归测试在精确 nominal、±0.005 rad 三种输入下均复现旧目标残留（测试注入旧目标差值 0.3 rad，该值不是现场测量）。

修复仅对 66 首球前校准增加最终目标发布：在进入第二次 R2 等待前明确发布 nominal；若输入新鲜度门禁丢弃此次发布，等待恢复后重试，直到 Python 发布序号递增。该序号只证明发布路径执行，不是原生接收/电机到位 ACK。随后仍保持首球前不推理、不发布新目标；首条有效 MOVE 后恢复原 V9，198 和开局后的校准行为保持原样。此遗漏不能被认定为此前摔倒或漂移的已证实原因。

本地 `test_onboard_active test_full_relay test_active_recovery test_active_continuity` 共47项通过，覆盖新增边界、发布丢弃重试、无球等待和首条 MOVE/HIT。旧版失败记录：[hold_commit_before.log](device_context/first_ball_idle_20260909/hold_commit_before.log)；修复后：[hold_commit_after.log](device_context/first_ball_idle_20260909/hold_commit_after.log)。这轮未重复运行设备推理测试或物理测试，以免占用他人调试资源。

19:24 已按明确清单备份并同步三设备独立 Yichao 目录，每台3文件（入口、测试、对应哈希清单），远端基线无分歧，上传后 SHA256 和本地设备副本一致。工作站哈希清单保留其独立依赖记录；机载各自清单保留设备差异。见[同步证据](device_context/first_ball_hold_commit_20260909/sync.json)。本轮没有启动、停止或重启任何现场程序，没有修改 Fixed＋V11，没有发布控制消息。待用户交接后，仍需实际验证66首球前站立及首条 MOVE 的切换，不自动恢复测试。

## 执行行为

66完成两次R2后保持最后一次校准写入的电机关节目标。在第一条有效MOVE被机载receiver实际受理之前，入口持续更新真实姿态、关节、历史和ACK，**不调用student推理，不发布新的关节目标**。这样不再让低层网络在尚无来球时自行根据HOLD参考生成步态。底层原生驱动仍保持原有PD目标；这不是断力矩，也不等于已经验证机器人在外力下绝对不位移。

66启动reference改为实测当前位置的HOME_HOLD，保留左手镜像。198的启动参数和执行流程保持原样。MOVE仍由原Planner在实际有效来球、模型/过滤及协议处理后发送；仅原始球消息、无效或过期命令不会解除等待。

第一条MOVE受理后，立即使用对应的新观察进入原V9推理循环；该MOVE的目标不被固定或覆盖，首拍可以正常横移、HIT。之后不再执行本次初始等待，包括后续接球、让位、RETURN及已开始运行后的重新校准。等待前的R2重新校准仍可用；用户退出仍可中断等待。没有新增等待超时。

本次没有改actor/filter、Planner候选目标、MOVE/CLEAR/HIT/RETURN协议、训练模型、冻结scheduler或镜像数学。

## 日志与就绪含义

等待来球时控制阶段标为`policy`，表示已完成R2且可受理Planner指令，同时明确输出：

```text
control_reason=waiting_first_ball_holding_calibrated_pose
```

这时student尚未执行。首条有效MOVE受理后记录`control_reason=first_prepare_accepted`，随后正常推理。Planner的raw_input会保存上述原因和真实遥测；机载动作recorder在进入正常控制循环后才开始写入动作帧，不伪造待命期间的模型动作。

## 验证与同步

本地58项相关测试通过，包含：12秒无球等待无推理/新关节目标、无效/过期MOVE不解除等待、有效MOVE仍可将66从−0.20m规划到−0.65m并正常HIT、后续校准不重新进入初始等待、等待时R2及用户退出。真实actor/filter＋冻结V9 scheduler的6拍文件回放，使用66 HOME_HOLD初始状态，仍按66→198交替完成；注入6秒交接延迟和6.7cm未到位，最终COMPLETE且fault=null。

工作站5文件、两台机载各3文件，逐设备比较基线后备份、写入并核对SHA256，本地设备副本同步更新。证据为 [同步清单](device_context/first_ball_idle_20260909/sync.json)、[验证记录](device_context/first_ball_idle_20260909/validation.json)。工作站依赖清单保持其原有版本。没有待同步源码。

同步后工作站相同58项测试及6拍回放通过；两台`run_onboard_active.py --check`均通过、没有打开总线。备份位于各自独立部署目录的 `backups/right_first_idle_r2_20260908T183404Z`，保留了本次替换前的文件。

同步时尚未重启控制程序。用户随后明确要求“重启测试吧”，已保留双机原生、198 Python及输入，重启66 Python为PID7988。新日志目录为工作站原会话下`idle66_20260909_023441`，66到达waiting_r2，198仍为POLICY，常驻就绪等待已启动。运行中的198没有热加载新入口。

## 用户取消测试与停止

用户随后表示太晚离场并明确“停止吧”。02:37–02:38完成本任务退出：

- 工作站就绪watcher及waiter、输入owner、Predictor及姿态转发均退出，保存窗口输出后移除`yichao_active_inputs`和`yichao_v9_active`。最终无command发布者；任务目录只剩两个交互shell，没有控制或输入进程。保留共享rosmaster。
- 66的Python PID7988和原生g1 PID3913分别收到SIGINT并退出，核对无本任务控制进程残留。
- 198有线口最初No route to host，管理口首次超时；再次管理口SSH成功时，本任务Python和原生g1均已不存在。本轮未成功直接向198发送停止信号，不能把退出原因断言为该信号或断电。

最终软件控制已停止，物理断电/断力矩未验证。新目录没有`planner.jsonl`，此次启动停在就绪等待阶段，没有新Planner击球记录；等待日志、窗口输出、此前Planner分片和机载记录均保留。停止证据见`device_context/first_ball_idle_20260909/stop_right.json`、`stop_left_retry.json`、`stop_workstation_final.log`及`stop_verified.json`。

已同步的首球待命修复保留，但本轮未完成物理站立或击球复测，不能将软件测试视为摔倒原因已排除。用户已取消测试，不自动重启。
