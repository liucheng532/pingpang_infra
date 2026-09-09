# Yichao V3→V9：基准后的实时接入进度

本记录承接[2026-09-07 基准](YICHAO_V3_V9_BASELINE_2026-09-07.md)，不覆盖其中的冻结归档。模型仍为 actor 199＋safe_filter_v3，安全过滤启用，真实闭环尚未验收。

**2026-09-08 完整 relay 更新：**工作站独立代码已完成两拍软件回放，MOVE/CLEAR/HIT/RETURN 与匹配反馈接通；[最新基准及证据](YICHAO_FULL_RELAY_PROGRESS_2026-09-08.md)。机载副本尚未同步该版，真实输入和 active 未验收；朋友测试期间不启动本任务机载程序或控制客户端。下文 98 项测试及移动接口进度是前一阶段记录。

**2026-09-08 更新：**移动专用执行接口已实现，本地及工作站各 98 项测试通过；两台独立机载目录的准备进度不同，机载 shadow 尚未运行。用户说明换电池后朋友将进行测试，当前不启动本任务机载程序。最新代码、缺项与回退以[机载准备记录](YICHAO_ONBOARD_MOVEMENT_PROGRESS_2026-09-08.md)为准。

**23:10 左右更新：已实现并部署 V9 状态/Predictor → 36D/85D → actor 199＋filter → `stage` 移动预览的诊断 shadow。现有启动器支持 `--planner yichao --mode shadow`，只读、20 秒、文件输出；active 仍关闭。** 本地 83 项测试通过；工作站新增 21 项相关测试通过。实际观察收到 monitor 583 次，双机 `/state` 均缺失，因而真实推理决策 0、控制命令 0；两台随后未查到 deploy_policy/g1_control 进程。详情、诊断假设、剩余 relay 集成和回退见[本轮接入记录](YICHAO_V3_V9_RUNTIME_SHADOW_2026-09-07.md)。下文各阶段的“未支持 shadow”等描述均为历史状态。

**22:35 更新：用户已提供并指定使用 Yichao 的 `unitree_description.zip`，已复制至[项目资产目录](../pingpang_assets/yichao_unitree_description_20260907/)，SHA256 校验通过，G1 URDF 有 29 个活动关节。此前请求训练 URDF 的缺项已由该用户指定资产解决，不再要求用户重复提供。**

**最新更新（22:11 后）：用户挪动左侧机器人后，rigid 1 的消息已恢复，两路位姿记录均通过新增 torso 转换模块。下面 22:03 的零消息是历史快照，已不再作为当前缺流结论。** 仍未录到完整挪动过程，不能据此认定物理对应关系已完成单机动作验收。

## 已完成：双机 jointpos 到工作站的只读通路

新增 `yichao_v3_v9/src/yichao_v3_v9/joint_feedback.py` 与 `tools/consume_joint_feedback.py`，已部署到工作站独立目录 `/home/odl/codebase/yichao_v3_v9_e14fd5b`。

- DDS 原始 q/dq 按冻结 V9 `joint_mapping.py` 转成 Lab 顺序；原 V9 `/state` 的 q/dq 已是 Lab 顺序，不重复重排。没有对 66 再做关节镜像。
- 保留机器人原始 tick、接收/发布时间和工作站接收时间，不跨机直接相减。
- 验证机器人角色、向量维度、有限值和四元数；重复/倒退序号、时钟倒退和会话改变会锁存该流故障。
- 输出是物理关节角（rad），尚未归一化。`real_input_accepted`、`sensor_age_verified`、`command_ack` 均为 false，不伪造 phase、目标确认或同时采样。
- 6 项新测试本地和工作站均通过，涵盖解剖关节映射、V9 不重复重排、时钟域保留、重复与重启、双机缺失/过期及非法数据。

2026-09-07 22:02 左右（北京时间），执行一次 20 秒双机只读采集：

| 项目 | 66 | 198 |
| --- | --- | --- |
| 工作站有效关节记录 | 970 | 985 |
| 机载 reader 退出码 | 0 | 0 |
| 机载无效数值记录 | 0 | 0 |

工作站转换拒绝数 0、退出码 0。两台复用本任务先前构建的 `read_lowstate_v2`，只订阅 DDS `rt/lowstate`。本轮未编译、安装或启动 motor controller，未发送控制命令。

此次传输为 **机器人 DDS → SSH 经本地开发机转发 → 工作站 stdin**，用于接通和验证真实数据，不是已通过 20 ms 预算的生产控制通路；不能把约 50 Hz 留存记录率当作已验证的端到端时延或传感器刷新率。

证据：[资源与输入预检](yichao_v3_v9/joint_transport_preflight.json)、[采集及测试目录](yichao_v3_v9/joint_transport_20260907T140202Z/)。工作站原始输出为 `output/joint_transport_20260907T140202Z.jsonl`。该证据目录也保存了修改前的工作站 manifest、依赖 lock 和 freeze 脚本；最终发布身份见 `joint_feedback_release_verification.json`。

## 22:03 时的卡点：第二台 torso 输入

当前在用 Predictor 对应项目的磁盘标定配置为：

```text
/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/calib/calibration_config.json
rigid 0 → double_right_robot_tracker → table_right / 66
rigid 1 → double_left_robot_tracker  → table_left  / 198
```

配置 SHA256：`b66e1ae1204662698f8f3f90542ce2731003c219dc3508e594fa788eda680fc4`；配置 `updated_at=2026-09-07T16:55:03+08:00`。用户随后表示“貌似 0 为右侧，1 为左侧”，与配置相符。这是**配置映射＋用户初步说明**，尚非逐台物理动作识别；不要继续把旧手册 `0→198、10→66` 当成当前实测映射。未修改现场标定，也未证明磁盘配置与运行进程内存逐项一致。

源码中的 `/debug/racket_rigid1_pose_origin` 实际承载 rigid 1 的原始位姿，字段是 `[source_time, frame, sensor, xyz(3), quaternion_xyzw(4)]`。虽然名字叫 racket，不能仅凭名字决定其物理对象；它也不是已转换好的 torso，不能直接填入 base 位置。

22:03 左右进行 10 秒只读 ROS 订阅：

- `/torso_pose_origin` 收到 2938 次回调，留存 471 条，最大接收间隔约 6.47 ms。
- `/debug/racket_rigid1_pose_origin` 有注册发布者，但收到 **0 条消息**。
- 该结果仅说明选定 ROS 通路当时缺少第二路数据，不能证明动捕软件没有 rigid 1，也不能从注册发布者推断数据正常。

证据：[标定文件快照](yichao_v3_v9/joint_step_calibration_sources.json)、[两路 torso 采样](yichao_v3_v9/torso_source_20260907T140335Z/)。新增 `capture_existing_inputs.py --topic <现有 topic>` 可限制只读采样范围；此次只订阅这两路，没有重测或重启 Predictor。

下一步先确认动捕软件里 rigid 1 是否正常跟踪，再沿其现有消息出口补齐第二路位姿；后续仍需共同坐标、历史与时间处理、关节预处理、真实输入推理及命令 ACK。第二路 pose 缺失时不启动双机 Yichao active。当前所有本任务短时 reader/consumer/ROS observer 均已正常结束；后续启动前仍需重新核对现场。

## 22:11 后复查：rigid 1 恢复与 torso 转换

用户表示已挪动左侧机器人。再次只读订阅 15 秒，主 `/torso_pose_origin` 收到 4405 次回调，rigid 1 debug topic 收到 4239 次回调；队列丢弃和发布者不匹配均为 0，最大接收间隔分别约 6.82/8.48 ms。两路留存 707/702 条。采样时 rigid 1 已基本静止，因此只确认恢复收到数据，没有观测完整搬动轨迹，也没有证明先前丢流原因。无需继续要求用户确认“是否存在 rigid 1”才能推进转换代码。

新增 `src/yichao_v3_v9/pose_feedback.py`、`tools/convert_pose_recording.py` 和 6 项坐标转换测试，已同步工作站并通过测试。只读复制现场标定至独立 `config/pose_calibration_source.json`，SHA256 与前述配置一致；未改变现场文件。

- 主 torso 已转换好，原样保留，不重复应用 tracker 或 table 变换。
- rigid 1 原始位姿按 `T_origin_tracker × inverse(T_torso_tracker)` 转为 torso；未重复应用 table 矩阵。
- 保留两路不同的源时间含义：主 torso 的 Predictor ROS 处理时间、rigid 1 的青瞳源时间。后者比工作站 wall time 约快 111.93 秒，仍不以接收时间覆盖源时间。
- 校验预期发布者、frame、sensor、四元数、顺序和固定 0/right、1/left 配置；不生成 phase 或 ACK。

对这次真实记录的离线转换，66/198 各 707/702 条通过，拒绝 0。最后一条配置推导的 torso 位置（m，`mocap_origin`）分别约为 `[0.2061,-0.1425,0.7904]` 和 `[0.3276,0.6252,0.7768]`。这是 torso，不是 pelvis/base；未与前一轮 jointpos 跨时段拼接、未组装成有效 36D/85D 输入。

证据：[本次原始数据、转换结果与验证](yichao_v3_v9/left_robot_movement_20260907T141150Z/)。工作站数据在 `output/left_robot_movement_20260907T141150Z.jsonl`，转换结果在同目录 `_torso_converted.jsonl` 文件。该阶段发布 manifest SHA256 为 `b8ab7e29c62c8ff49fa2653305bdf93538320311dc627a378594ed072424e634`。

下一步是让双机关节与两路 torso 同期采样，按冻结 V9 pelvis 估计逻辑组合，再完成训练坐标、历史/源龄、关节归一化和目标记忆验证；随后才能验收 Yichao 真实输入推理。在线发布和明确 ACK 仍未实现，未启动机器人控制。

## 22:24 同期采集与 base 换算

新增 `base_feedback.py` 与 `tools/assemble_base_recording.py`，已部署工作站，6 项测试通过。将同期记录的两路 torso 与同一机器人最近的已接收 q 配对，要求接收时间差不超过 50 ms，并使用冻结 `G1PelvisPoseEstimator` 换算 pelvis/base；没有镜像两次，也不把未来关节样本配给旧 pose。

本轮两台 DDS 只读 reader、ROS observer 和工作站关节 consumer 均正常退出。成功换算 66/198 各 805/798 条；启动时 66 条没有前序关节样本、18 条接收时间差超过限制，均明确拒绝。成功样本最大接收时间差约 47.63 ms。18 条时间差超限说明本轮 SSH 诊断转发尚有抖动，未宣布它达到控制链路的源龄/20 ms 性能验收。

末条 base 位置（m，mocap_origin）为 66 约 `[0.2102,-0.1514,0.7476]`、198 约 `[0.3312,0.6342,0.7336]`。这些是同轮记录的诊断换算，物理采样时间对齐与训练共同坐标仍未验收。证据：[同期采集目录](yichao_v3_v9/base_capture_20260907T142429Z/)。没有要求用户按 R2 或进入 default pos，没有启动本任务机器人控制。

同时新增本地 `ball_feedback.py`，从现有 Predictor monitor 的一条 JSON 提取当前球位置/速度、击球点/速度、拍速、TTS，避免独立 topic 混帧。6 项本地测试通过；截至此段记录，该模块尚未完成工作站部署和真实来球验收。保留 waiting→tracking 的本会话 shot 身份，启动在半拍、重复/倒退消息、过期 TTS、disarmed/held 输出不会被当成有效新 shot。这个身份不是机器人 token/ACK。

## 用户指定 URDF 的来源确认

归一化资产核查曾在工作站找到另一个 calibration URDF，与 Controller 候选的 28 个关节限位不同，因此未任选候选代入。用户随后提供的正式指定文件优先使用，历史比较仅保留在 [候选核查目录](yichao_v3_v9/joint_normalization_candidates/)。

项目压缩包、提取出的 G1 URDF、SHA256 与来源说明见[资产 README](../pingpang_assets/yichao_unitree_description_20260907/README.md)。软限位计算公式已对照本地 Isaac Lab 源码：`midpoint ± 0.5 * (upper-lower) * factor`；Controller 匹配配置 factor 为 0.9。下一步将此指定 URDF 的关节表按模型 ABI 重排，生成并验证正式归一化参数，取代仅供合成夹具使用的 ±2 限位。

随后已生成 `config/joint_normalization.json`，独立适配包内保存指定 URDF 至 `vendor/training_assets/unitree_description/urdf/g1/main.urdf`。新增 `joint_normalization.py`：运行时校验 URDF 哈希、29 个 joint names、硬限位表和 0.9 软限位计算。3 项测试对照冻结 `IsaacV0ShotPlannerEnv._joint_position_normalized`、中点/边界与非法输入，全部通过；更新后关节流原有 6 项测试也通过。

关节反馈保留物理 `q_lab_rad` 供 FK，另输出 `normalized_joint_position_v3` 供过滤器，避免把归一化关节值传给机械几何计算。合成 fixture 的 ±2 参数仍单独保留以维持既有参考测试，未用它替代本次指定 URDF。真实全链路输入仍未验收，不因归一化子模块通过就启用 active。

工作站同步已完成：归一化 3 项、关节反馈 6 项、ball feedback 6 项测试均通过；9 个同步文件逐一校验与本地一致。`ball_feedback.py` 此时已部署，更新前述“尚未工作站部署”的阶段状态；真实来球验证仍待完成。证据：[URDF/归一化工作站验证](yichao_v3_v9/urdf_normalization_workstation_verification.json)。该阶段 manifest SHA256：`7d4b49d0e0763da7dfe8efcb39766dca26010831467c9967a48151bc27fcfbaa`。工作站只同步 G1 URDF 和适配代码/参数，完整压缩包留在本地项目资产目录；现场 Controller、标定与限位配置未修改。

22:44 后读取现有 Predictor monitor 8 秒，232 条留存消息经工作站 `BallFeedback` 解析，错误 0；全部处于 `waiting`，有效来球字段状态为 0 次。没有把等待状态的占位击球点/TTS 当作有效 shot。此结果只验收本次实际消息的等待分支，不代表来球、完整特征或推理闭环通过。证据：[球接口采样](yichao_v3_v9/ball_interface_20260907T144459Z/)。本次 observer 正常结束，未重启 Predictor 或发布控制命令。
