# 真实来球触发 Yichao 决策的 shadow 记录

会话 `v9_20260908_235934_09016b`，由用户通过两个独立入口启动并现场发球。工作站会话名日期仍为 2026-09-08；本记录于跨日后归档。此次仍是无电机输出的 shadow，不能作为 active 或完整 relay 验收。

## 实测结果

- 收到 20 条有效球预测，均属于同一 `mocap-1` shot；另外收到 1162 条无效预测。20 条不是 20 拍。
- Actor 199 对该 shot 生成一组横向目标；3 条 relay_step 重复记录同一决策，不是三次独立规划。
- 目标为 66/物理右侧 `-0.7903623581 m`，198/物理左侧 `+0.2455353886 m`，均为共同坐标的绝对 Y，不是移动距离。
- 安全过滤器评分 `0.0089687137`，阈值 `0.3905482218`；nominal 通过，filter 和解析投影均未改动目标。该评分是模型输出，不是实机碰撞概率或实机安全验收。
- 记录 5 次 MOVE 协议发送，只有 2 个唯一 sequence/token（66、198 各一个），并非 5 次独立动作。现有 summary 的 `retransmissions=0` 不足以描述这些重复发送；后续应核对计数语义。
- relay 最后一条摘要仅列出 198 已受理。后续核对完整 raw_input，66 和 198 均有匹配 sequence/token 的 accepted 反馈；不能把摘要的局部状态当作 66 未受理。确认只表示 shadow 接收器的协议应用，不表示机器人实际移动或到位。
- 未发送 HIT，未完成任何拍次或交接。随后因 `runtime_input_or_execution:base_history_gap` 结束。

## 未解决事项

`runtime_shadow.py` 中此错误表示组装位置历史时发现接收样本间隔超过 50 ms。已定位触发样本为 66：工作站接收间隔 55.311 ms，机器人发布源时间间隔 19.464 ms，sequence 差值为 1。这表明接收时间抖动，并不能证明物理动捕丢帧。证据见[历史间断分析](yichao_v3_v9/user_two_terminal_20260909/history_gap_analysis.json)。

独立工作站 `tools/run_movement_pilot.py` 的状态/球订阅和命令发布此前未显式启用 TCP_NODELAY，而 clock 通道已经启用。现已补上，保留原 50 ms 间断检查及全部有效性门禁。此改动针对可能的 TCP 批量延迟；尚无修正后的现场测量，不能宣称已经解决抖动。源时间历史对齐仍未验收。

本地 13 项输入测试、4 项双入口测试通过，工作站 13 项输入测试通过。首次双入口测试因未设置 PYTHONPATH 导入失败，使用项目 src 后通过。同步前确认远端源码和 lock 中旧哈希与基线一致，远端备份、逐文件 SHA256 及本地设备镜像更新完成，详见[同步清单](device_context/nodelay_20260908T160829Z/sync.json)。本次只同步工作站 client 和该设备 manifest；两台机载不运行这个工作站 client，无机载业务文件变更。

本次证明当前输入链路可以在真实来球下触发 actor＋filter 和 MOVE 协议，并纠正“当前始终收不到有效球”的判断。此前无效球的原因仍未定位；实时历史、双机确认、源龄及 active 门禁均未验收。

## 证据

- [会话采样统计及结束原因](yichao_v3_v9/user_two_terminal_20260909/v9_20260908_235934_09016b_summary.json)
- [决策、MOVE、受理及总结原始事件](yichao_v3_v9/user_two_terminal_20260909/valid_ball_decisions.json)
- 工作站完整记录：`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/sessions/v9_20260908_235934_09016b/`。

上述后续修正仅涉及独立工作站 client，不改模型、原方案、现场标定或依赖。active 尚未开放，未启动电机控制。
