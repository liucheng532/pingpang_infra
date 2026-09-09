# 2026-09-08 晚间双机 V9 实际 shadow

用户已明确两台空闲，可供本任务测试。现场物理左侧为 198、右侧为 66。两台独立机载环境已补齐；本轮实际运行冻结 V9 i19000 shadow，不再是仅在工作站用合成执行器测试。**尚未通过真实输入/闭环验收，active 仍关闭。**

## 已执行与隔离

工作站 `/home/odl/codebase/yichao_v3_v9_e14fd5b`；两台 `/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z`。独立被动 DDS 遥测向本机私有 `239.255.77.{66,198}:7767` 发布状态，V9 shadow 的 LCM 包装器禁止任何发布；没有启动 g1_control、default、校准或电机指令。每轮使用唯一 `/yichao_v3_v9/<session>/…`。

原 Predictor 当时未运行、输入及控制 topic 无发布者，故从经过哈希核对的现场文件建立独立 `predictor_snapshot_20260908`，在本任务环境限时运行；未修改原 Predictor/Runtime/deploy、共享依赖或网络配置。快照来源为 Predictor `302eaf5` 加现场修改，采用当天 16:45 更新的标定，rigid 1→198、rigid 0→66；这只是当前配置，物理/坐标正确性仍须现场确认。

## 第一轮：live_v9_20260908a

- 两台被动遥测 45 秒，DDS 包校验拒绝数均为 0；两台 V9 shadow 约 40 秒，正常退出。工作站 relay 20 秒，退出码 2 表示未完成拍次，fault 为空。
- 工作站接收 66/198 状态 966/972 条，约 48.38/48.74 Hz，客户端队列丢弃 0。
- 双机 DDS 接收龄中位数约 3.6 ms；torso 本机接收龄中位数约 11 ms，最大约 169 ms。这些不是动捕硬件采样到控制端的端到端延迟。
- 首次可用时钟区间在运行约 13 秒后才出现，66 还出现短暂失效；不能据局部区间宽度小于 20 ms 宣称时钟链路通过。
- 5866 条球消息均无效。Predictor 日志反复报告没有球刚体 30300 回调；没有真实来球推理，没有下发命令或 HIT，没有构造 ACK。
- pelvis 中位数：66 `[0.0474,-0.0496,0.5968] m`，yaw `142.51°`；198 `[-1.2167,1.4340,0.8522] m`，yaw `-4.49°`。198 的 Y 超出训练目标工作区 ±1.20 m，双机朝向也不一致。需要区分摆放尚未就位与标定/刚体映射问题，不通过偏移、交换角色或扩大工作区绕过。
- 私有 Predictor 的 `live_shadow_20260908a` 到时后 SIGINT/SIGTERM 未能结束，启动器对其确切子 PID 2428863 执行 SIGKILL，退出 -9；已核实 PID 不存在。不能记作正常退出。日志还出现约 -108 秒的 tracker age，硬件/主机时钟语义仍待核清。

## 针对实测问题的代码更新

工作站和机载时钟通道启用 `tcp_nodelay`；工作站记录每次 probe、连接数、reply 到达/处理时间和 nonce 匹配，机载 state 增加缺少区间/租约过期的具体原因。尚不将这些修改当作延迟问题已解决。Predictor 启动器增加停止原因和确切信号序列记录。

7 项移动事务/时钟边界测试通过，修改文件通过 Python 3.8 语法检查。已逐文件备份、同步到对应设备并核对 SHA256，也更新本地设备副本。同步证据：[clock_diagnostics_1788864705740264562](device_context/clock_diagnostics_1788864705740264562/)。

原始运行日志、输入记录、当前 Predictor 快照清单和第一轮统计：[live_shadow_evening_20260908](yichao_v3_v9/live_shadow_evening_20260908/)。后续应使用新 session，不能覆盖本轮文件。

## 第二轮与本地 context 分析

`live_v9_20260908b`：两台 V9 shadow、被动遥测和姿态转发正常退出；工作站仍无有效球、命令 0、fault 为空、队列丢弃 0。首次时钟就绪缩短到约 0.48 秒。两台各 39 次回复全部匹配，区间可用状态分别 922/974 和 950/974；66 仍有 28 条状态报告租约过期。去除机器人回复处理时间后的 RTT 中位数约 7.2 ms、最大约 150/142 ms，超过 20 ms 的交换为 5/3 次，因此仍未通过连续实时性验收。没有放宽 20 ms 不确定度或 1 秒租约。第二轮私有 Predictor 到时仍需要 SIGKILL 退出。

以用户提供的三个本地设备目录为依据，逐文件比较两台 V9 与冻结副本：`g1_pelvis_pose.py`、`joint_mapping.py`、`left_right_mirror.py`、`planner_ros_bridge.py`、`lcm_agent.py` 五项均逐字节相同。当前位置/朝向差异不能归因于两台这五个文件版本不同。Predictor 使用 `inv(T_world_origin) @ T_world_tracker @ inv(T_torso_tracker)` 得到 torso；机载用物理腰部三个关节做 torso→pelvis FK，发布 state 时再转换为 LAB 关节顺序。Yichao 双机 torso 转发没有重做表格或 tracker 标定。

发现原 Predictor 的真实输入问题：`Mocap._run_torso_frame` 丢弃缓存中的源时间，用新的 `job.frame_ros_time_sec` 发布旧 tracker 值；仅靠 ROS topic 频率/序号不能识别动捕停止。

本任务新增 `src/yichao_v3_v9/predictor_freshness.py`，在独立启动器导入 TableTennis 前扩展其 Mocap 类；不修改原 Predictor 文件或私有冻结原始快照。扩展校验源 frame/timestamp/有限位置/非零四元数，拒绝重复和乱序，保留源时间、回调 monotonic 和处理时间；torso 队列中超过 0.25 秒的缓存不再重发，ball 缓存及工作队列也检查接收龄。生成独立 `mocap_source.jsonl`。这是回调新鲜度保护，仍不证明相机采样龄或跨设备时钟对齐，pose header 仍明确为 Predictor 处理时间。

4 项新增测试在本地和工作站均通过；还用本地设备副本的实际 ctypes Mocap 回调验证了重复帧不覆盖缓存、原始源时间保留、球缓存到时失效，SDK 没有启动。源码/测试/启动器已备份同步到工作站并更新本地设备副本，证据：[predictor_freshness_1788865028555281877](device_context/predictor_freshness_1788865028555281877/)。

尝试 `source_guard_20260908c` 限时实测时，启动检查发现新的 Predictor `/table_tennis_predictor_2447500_1788864879303` 和向双机 command 发布的 `/doubles_v9_real_fixed_relay`，因此拒绝启动；未运行新 Predictor、未停止现有程序。该扩展的真实动捕运行验证尚待完成。已向用户确认当前程序运行者，不能沿用此前空闲快照。新发布者与本任务前两轮 Predictor PID 不同。

## 用户确认与暂停交接

用户确认新的 Fixed Planner/双打程序由其本人执行 `python3 -B scripts/run_doubles_stack.py --shadow` 启动，用于熟悉现有双打流程。因此本轮只确认存在 command 发布者，不将其描述为用户已启动 active 或机器人正在运动。

用户明确要求：不停止、接管或修改这套程序；不并行启动向相同 command topic 发布消息的 Yichao 程序；先保存进展，待用户完成现有双打操作并明确交接后再继续。Yichao 实机接入已暂停，本次仅更新本地记录，没有执行远程命令或进程操作。

保留的工作包括双机 V9 shadow 运行记录、时钟诊断修正、工作站独立 Predictor 新鲜度扩展、对应测试及逐文件备份/同步证据。新鲜度扩展尚未完成真实动捕运行验证；active 仍未开启。恢复时先重新核对现场状态及交接范围，不沿用此前空闲快照。
