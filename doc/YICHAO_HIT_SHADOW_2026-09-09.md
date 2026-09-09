# 更新接口后的真实来球 → V9 HIT 受理

用户运行会话 `v9_20260909_001454_a68f60`，使用 actor 199、safe_filter_v3 和配套 V9 i19000，模式为无电机输出的 shadow。

## 本轮证据

- 一次 `mocap-1` shot，共 215 条有效预测；不是 215 拍。
- actor 目标按训练槽位为 66/物理右侧 −0.786372 m、198/物理左侧 +0.306190 m，均为共同坐标绝对 Y。
- filter 评分 0.00490999，nominal 通过；没有 filter/解析投影介入。评分不等于真实碰撞概率。
- 双机 MOVE 均受理；随后 66 CLEAR 受理，再发送 198 HIT。HIT 首次反馈 phase=HIT，选中 hit motion 27。
- 记录 8 次协议发送，但只有 4 个唯一 sequence/token：2 MOVE、1 CLEAR、1 HIT。HIT 两次发送使用同一 token，不是两次独立击球；summary 的 retransmissions 计数语义仍有待修正。
- 没有 RETURN，没有完成交接。66 实测 Y 约 −0.144 m，198 约 +0.361 m，未到交接目标 ±0.9125 m。shadow 下不发布电机指令，不能要求不运动的机器人产生实际到位反馈。
- 198 的 MOVE 曾返回 completed：当时目标 +0.30619 m 与测量 +0.36130 m 的差约 5.51 cm，处于已有 6 cm 完成容差内且满足稳定条件。此反馈不能当作发生实际移动的证据。
- 本轮无 base_history_gap 退出，但并未证明所有接收间隔都小于 50 ms：198 最大间隔仍约 51.86 ms，66 约 36.18 ms；双机记录均未出现 sensors_recent=false。时钟未就绪记录仍存在，不能据一次推进通过宣布实时性验收。

终止原始错误为 `runtime_input_or_execution:latched_shot_prediction_expired`，发生在 COMMITTED 等待到位期间。工作站及双机本轮程序已结束；原始退出结果保留。

## 后续修正

修正 `RuntimeRelay` 的轮询输入依赖：PREPARING/COMMITTED/RETURNING 只需要当前机器人状态、反馈及事务超时，不再要求旧球预测维持新鲜。CLEARING 在发 HIT 前仍检查本拍预测新鲜度；选择新的 CLEAR/RETURN 仍组装最新安全特征。交接超时仍为 HIT 后 5 秒，没有取消或扩大超时。

新增测试证明旧球超过 5 秒但 HIT 尚未满 5 秒时，可以继续等待真实反馈，不能伪造完成；超过 HIT 交接期限则正确报 handoff_timeout。本地/工作站 8 项事务测试通过，真实模型＋冻结 scheduler 的两拍合成回放仍完成、每机一次 HIT。修改后未运行现场程序，active 未开放。

## 记录

- [本轮汇总和唯一命令](yichao_v3_v9/user_two_terminal_20260909/v9_20260909_001454_a68f60_result.json)
- [首次受理反馈、位置范围及接收间隔](yichao_v3_v9/user_two_terminal_20260909/v9_20260909_001454_a68f60_feedback.json)
- [后续修正同步清单](device_context/posthit_feedback_20260908T161934Z/sync.json)
- [工作站测试](device_context/posthit_feedback_20260908T161934Z/remote_tests.json)
- 工作站完整会话目录：`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/sessions/v9_20260909_001454_a68f60`。

本轮已验证真实来球到模型、安全筛选及 V9 MOVE/CLEAR/HIT 受理；不能据此宣布物理动作、闭环交接或 active 验收。无需让用户重复静止 shadow 来等待物理到位。
