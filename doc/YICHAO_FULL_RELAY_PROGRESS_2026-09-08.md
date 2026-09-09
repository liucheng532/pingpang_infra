# Yichao V3→V9 完整 relay 软件接入基准

后续现场证据见[同期输入核对](YICHAO_LIVE_INPUT_CAPTURE_2026-09-08.md)：双机状态与有效球预测已录到，但朋友当前运行的是 V11 resume42500。该录制不能替代本任务 V9 机载验收；等待设备交接。

2026-09-08：actor 199＋safe_filter_v3 已接入 MOVE→CLEAR→HIT→RETURN 的专用命令/反馈链路，本地和工作站的两拍软件回放通过。**真实输入验收、机载运行和 active 尚未完成。** 用户说明两台机器人换电池后已开机，朋友将测试；当前不启动本任务机载程序或控制客户端，不停止、修改或挤占朋友的进程。

同日后续修正：交接不再复用击球前整份 85D 特征。`handoff_features` 使用新鲜双机位置历史与关节角，保留本拍 strike target/racket velocity 并更新 TTS；新球缺失不阻断已有交接，但不会允许新 HIT。过期机器人状态仍拒绝。协议诊断日志保存评分所用 85D、采样时间及来源。V3 filter 未在中途协议阶段标定，CLEAR/RETURN 评分仍只用于诊断；该修正没有新增实机安全验收结论。相关 18 项测试通过（0.720 秒），更新后的两拍本地回放仍为 285 条记录、双机各一拍、COMPLETE。证据保存于 `yichao_v3_v9/full_relay_20260908/fresh_handoff_*`。

## 实现和验证

- `command_receiver.py`：完整字段检查、session/shot/token 去重、scheduler 副本预应用、目标锁存验证和稳定完成反馈。处理了新测量位姿与 scheduler 前一周期 reference 位姿不一致时的绝对 Y 目标校验；未修改冻结 scheduler。
- `runtime_relay.py`：将双机状态、球预测、36D/85D、模型/安全过滤与 relay 接通。初始目标记忆采用冻结 V3 reset 的 home 值，后续仅根据匹配的 prepare 应用反馈更新。输入/时钟/反馈不满足条件时不能发起新 HIT。
- `relay.py`：依据冻结配置的 locomotion preemption 与 80 ms reservation，准备目标受理后可在移动阶段提交；先取得同伴 CLEAR 的明确应用反馈，再向击球方发 HIT。同伴拒绝或超时会锁存故障，不能继续击球。RETURN 受理与实际完成分别记录。
- `run_onboard_relay.py`、`run_runtime_relay.py`：专用机载 shadow 接收器和工作站 relay 客户端，使用 `/yichao_v3_v9/<session>/…`。机载 LCM 包装器禁止发布；active 入口仍关闭。
- 启动选择器支持 `--planner yichao --mode shadow --relay-session <session>`。这会向私有接收器发出模拟调度命令，区别于不带 `--relay-session` 的只读诊断；朋友测试期间不运行该客户端。`--dry-run` 只打印入口，不连接 ROS 或启动子进程。

工作站使用 `/home/odl/codebase/yichao_v3_v9_e14fd5b/.venv`，原 Runtime/Predictor 均未修改。两台机载目录仍是较早准备版本，尚未同步本轮完整 relay，不可视为与工作站一致。198 私有环境导入和被动遥测编译已通过；66 环境补齐、编译及两台最终验证仍待设备窗口，详见[机载准备记录](YICHAO_ONBOARD_MOVEMENT_PROGRESS_2026-09-08.md)。

## 可复现证据

[工作站回放摘要](yichao_v3_v9/full_relay_20260908/workstation_replay.jsonl.summary.json)与[完整时序](yichao_v3_v9/full_relay_20260908/workstation_replay.jsonl)：285 条记录，198/66 各一次 HIT，MOVE 4、CLEAR 2、HIT 2、RETURN 4，最终 COMPLETE、无故障。本地同一入口结果一致。

这是合成位置轨迹驱动真实 actor/filter、专用接收器及冻结 scheduler 的软件验证；未运行机器人动力学闭环，未产生真实 ACK，不能据此证明移动或 RETURN 的实机稳定性。日志顶层明确 `real_input_accepted=false`、`control_network_access=false`。

本地完整测试集 102 项通过（5.153 秒）；随后补充同伴 CLEAR 拒绝禁止 HIT 的用例，并验证启动参数不混用回放输入，相关 15 项测试通过（0.582 秒）。证据日志存于同目录。工作站本轮完整回放输出及修改前备份在 `output/full_relay_20260908/`，模型仍采用冻结 e14fd5b / V9 i19000 整套资产。

文件离线复现命令（在独立包目录；无需 ROS 或机器人）：

```bash
env PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .venv/bin/python -B tools/simulate_runtime_relay.py --output output/full_relay_manual.jsonl
```

输出文件名须尚不存在；再次运行时换一个新文件名，不得覆盖既有证据。

## 尚未通过的条件

1. 同期双机真实状态、共同坐标/朝向和完整 36D/85D 输入仍需验收。接收时间栅格与动捕原始采样栅格不能混称；源时钟偏移也不能当网络延迟。
2. 被动 DDS 时间是机器人接收回调时间，尚不能证明硬件采样龄。历史 Predictor 记录也存在源时钟与工作站 wall clock 偏移，不能以新接收时间掩盖旧样本。
3. V9 的微小位移请求可能 no-op；专用接收器会拒绝未锁存目标，不会把它认作成功。原 scheduler 的物理稳定性和已知 66 RETURN 风险并未由软件回放解除。
4. 待朋友测试结束、控制权交接后，重新核对运行进程与资源，再同步并验收本任务机载 shadow。当前不需要用户为本任务操作遥控器。

回退使用本任务独立发布清单及修改前备份；不覆盖原方案，不自动重启他人程序。目录回退不撤销已经执行的机器人动作。
