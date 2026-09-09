# 2026-09-09 当前入口：连续 active 与自动日志

当前采用独立 actor 199＋filter＋V9 i19000。工作站 `/home/odl/codebase/yichao_v3_v9_e14fd5b` 两终端入口如下；已有本任务会话运行时不要重复启动。

```bash
# 终端 1
python3 -B scripts/run_yichao_stack.py start active
# 终端 2，等终端 1 ready 后
bash scripts/start_yichao_dual_robots.sh start active
```

首拍固定为右侧66（table_right），后续66→198交替；停球超过10秒重新开局仍由66先打。首拍前再次R2校准只等待，不把无动作的会话永久锁死。

66默认恢复原版镜像OUTWARD_HOLD启动，第二次R2后持续运行V9策略；已撤掉首球前暂停推理、保持校准关节目标的实现。用户现场确认原版能进入策略并站稳，但首球前移动仍未解决。

可选的动态HOME_HOLD待命候选只调整66的启动参考，使用原V9当前位置HOME_HOLD；第二次R2后仍运行完整50Hz观察、策略推理和关节控制。此处固定的是全局位置目标，电机关节命令由策略持续计算。有效MOVE/HIT、后续交接和198保持原流程；再次R2仍按所选启动参考重新校准。候选尚未实机验收，不保证无漂移。需在旧控制退出后的新会话中显式选择，不能对当前运行实例热切换：

```bash
# 终端1命令不变；终端2显式选择候选
bash scripts/start_yichao_dual_robots.sh start active --right-startup home-hold
```

省略`--right-startup`即使用原版OUTWARD_HOLD。两种方式均不等待首球才开始V9推理；状态`policy`只有实际推理成功后才设置。

active 默认持续运行（seconds=0、shots=0），无球等待。准备/球过期只跳过当前拍；交接超过 5 秒、计算超过 20 ms 不退出。V9 参考动作进入 HOLD 后按实际稳定位置继续交接，并记录目标误差。短暂输入间断恢复后继续；运行中 R2 是重新校准，不能恢复旧事务。

每次启动自动记录 `output/sessions/<session>/`：Predictor、poses、readiness、Planner console 和 `relay.jsonl`（直接运行 Planner 未指定输出时使用独立 `planner.jsonl`）。Planner JSONL 包含 36D/85D 输入、模型/过滤结果、命令/ACK/双机状态、跳拍/等待原因、耗时和退出信息；16 MiB 自动分片、后台写入。两台机载执行记录位于各自 `output/<session>/policy_<robot>/`，恢复运行会另有子目录。全项目详细记录：`doc/YICHAO_CONTINUOUS_PLANNER_2026-09-09.md`。

两台各按第一次 R2 校准，完成后再按第二次进入 policy；等双机 POLICY / Planner started 后发球。以下内容是早期诊断阶段的历史说明，不代表当前 active 入口状态。

# 2026-09-07 最新入口：诊断 shadow

工作站独立目录支持 `./run_planner.sh --planner yichao --mode shadow --subscriber-ip 192.168.123.165`。CPU 20 秒，只读 V9 双机状态与已有 Predictor，生成 36D/85D、actor 199＋过滤器输出和无效 stage 命令预览。不会发布控制命令；active 仍关闭。接收时间插值和原控制器目标对照输入不等于真实时序、Yichao ACK 或闭环验收。

本轮本地及工作站各 83 项测试通过。现场 monitor 583 次、双机 /state 缺失、决策 0；下文 shadow 尚未实现的表述为历史状态。详见项目 doc/YICHAO_V3_V9_RUNTIME_SHADOW_2026-09-07.md。

# Planner 工作站上的 Yichao 独立实现

已落地目录：`/home/odl/codebase/yichao_v3_v9_e14fd5b`。

用户于 2026-09-07 指定 Yichao 的 `unitree_description.zip`。独立包保存其 G1 URDF 至 `vendor/training_assets/unitree_description/urdf/g1/main.urdf`，哈希 `2efe5bddebd5a3ba4685a085031892ed14abc16b343970876380ae61a08d76cc`。`config/joint_normalization.json` 使用该文件和 Controller 的 0.9 soft-limit factor；`joint_normalization.py` 已通过冻结 V3 特征函数对照测试。关节反馈的物理 `q_lab_rad` 与 `normalized_joint_position_v3` 分开保存，FK 使用前者。`base_feedback.py` 已完成同轮双机记录的 base 换算，仍标记源时钟对齐和完整真实输入未验收。旧缺少训练 URDF 的记录已由用户指定资产解决。

2026-09-07 后续接入：新增 `src/yichao_v3_v9/joint_feedback.py` 和 `tools/consume_joint_feedback.py`，可将两台 DDS 原始关节或 V9 state 转为 Lab 顺序的 q/dq 并记录原始时钟。工作站 20 秒真实双机采集已通过，6 项相关测试通过；这一路暂经开发机 SSH 转发，只作诊断，不把接收新鲜度当成传感器源龄，也不发布命令。`capture_existing_inputs.py` 新增可重复 `--topic`，可只观察指定的现有输入。rigid 1 在 22:03 曾无消息，22:11 用户挪动左侧机器人后的采样已恢复两路数据。新增 `pose_feedback.py` 与 `tools/convert_pose_recording.py`，按独立标定副本将 rigid 1 原始位姿转为 torso，主 torso 不重复变换；707/702 条真实记录转换通过，6 项坐标测试通过。torso 尚未与关节同期组合成 base，真实输入仍未验收。完整交接记录在本地 `doc/YICHAO_V3_V9_PROGRESS_2026-09-07.md`；Yichao shadow/active 的关闭条件仍保留。

新增 Planner 选择入口（2026-09-07），只负责本次选中的 Planner，不启停 Predictor、Monitor 或机器人控制程序：

```bash
cd /home/odl/codebase/yichao_v3_v9_e14fd5b
./run_planner.sh --planner yichao --mode replay --input fixtures/two_shots.jsonl
./run_planner.sh --planner fixed --mode shadow --subscriber-ip 192.168.123.165 --dry-run
```

第一条实际运行 Yichao 离线链路，输出到自动创建的 `output/runs/<run_id>/`，包含启动身份、stdout/stderr、特征、原协议命令和转换后的 `v9_wire_preview`。第二条仅显示原 Fixed Planner 的启动选择，不启动任何程序。原 Runtime 的启动入口保持原样。

`--mode observe --subscriber-ip 192.168.123.165` 可启动 20 秒现有输入记录；它是诊断采集，不冒充 Yichao 决策。`--planner yichao --mode shadow/active` 当前明确返回缺项并退出：真实 36D/85D 输入契约未验收、专用执行 ACK 与在线发布尚未实现。选择参数已经实现，不代表真实接管已经完成，也不会偷偷回退成 Fixed Planner。

选择原 Fixed Planner 的 shadow/active 会调用现场原入口，因此两者都占用原 command topic。入口在启动前检查发布者，发现已有发布者就拒绝；本任务实例另用文件锁互斥。此检查不是跨机器的控制权租约，现场仍需明确交接。退出只转发信号给本入口启动的确切子进程。

`v9_wire.py` 已验证移动/让位/RETURN/HIT 的现有 V9 字段表达，编码前完整验证全部字段。默认禁用 HIT；回放显式允许编码 HIT，但不发布。原 bridge 的接收超时没有替代跨机源龄验证，`last_applied_sequence` 也不作为应用成功的 ACK。

这里包含实际源码、V3 actor/filter、整套 V9 固定依赖、596 条动作副本、独立 `.venv` 和回放入口。没有改动 `pingpang_doubles_v9_runtime`、Predictor 或机器人部署目录。

```bash
cd /home/odl/codebase/yichao_v3_v9_e14fd5b
./run_workstation_offline.sh \
  --input fixtures/two_shots.jsonl \
  --output output/my_run.jsonl
```

输出文件须不存在。默认 actor 199，追加 `--actor 190` 可对照。入口仅处理文件，单线程 CPU，进程 nice=19、磁盘 I/O idle priority；没有 ROS、LCM 或 active 参数。无需改现场环境变量或连接机器人。

当前 relay 执行 `prepare → peer-clear + HIT → outward 完成 → RETURN → 双机返回完成`，由命令身份及实际目标确认驱动。`protocol.py` 复用冻结 V3 的 outward/runway/返回目标投影方法；`executor.py` 调用原版 V9 scheduler 和 mirror。

V3 filter 的校准边界是每拍 actor 选目标时，因此该处风险阈值严格生效。中途 peer-clear/RETURN 的风险分数单独作为诊断记录，不冒充经过校准的判断；这些协议目标由冻结 V3 运动可行性及 workspace/gap 检查约束。当前仍拒绝真实输入，不能把合成闭环视为真机安全验收。

运行本目录的测试（全部产物仍留在本目录）：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 ionice -c 3 .venv/bin/python -B tools/test.py
```

结果位于 `output/test_results.json`。工作站环境为 Python 3.10、CPU Torch 2.7.0、ORT 1.23.2、NumPy 2.2.6，从现有 CPU 环境只读复制到 `.venv`；确切版本见本机 `requirements.lock` 和 `config/environment_provenance.json`。原环境未安装、升级或删除任何包。

新增真实消息格式的只读诊断入口：

```bash
./run_workstation_observer.sh --output output/input_preflight.jsonl
./run_workstation_observer.sh --replay fixtures/telemetry_synthetic.jsonl \
  --output output/input_fixture_check.jsonl
```

默认只查询本机 ROS master 的发布者和类型，不注册节点。五个必需输入有缺项时退出码为 2，具体 topic 写在输出中；类型表有名称不算存在发布者。当前工作站实测这五个输入均无发布者，因而没有启动在线订阅。

`src/yichao_v3_v9/telemetry.py` 解析 Predictor JSON、机器人 state JSON、torso PoseStamped；保存原始字段、接收时钟和源时钟，检查角色、维度、数值、四元数、时间、序号。`observer.py` 提供有界只读订阅，代码中没有 Publisher、推理或执行器调用。测试：`.venv/bin/python -B tests/test_observer.py`。

将来输入和资源条件具备后，短时诊断订阅可使用 `--capture-seconds 10 --subscriber-ip <工作站已配置且发布者可达的IPv4地址>`。入口限时 60 秒、日志上限 16 MiB、队列 32、每 topic 订阅队列 1；节点名随机唯一，关闭 rosout 发布。输入发布者改变、接收断流或输出故障会退出并注销本任务订阅。此参数不会配置网络，也不会启动任何控制器；当前未执行这一模式。ROS Python 代码已复制到 `vendor/ros1`，使用前校验哈希；依赖在自己的 `.venv` 中，不从现场 Runtime/Predictor 导入。

诊断通过只表示消息字段通过检查。真实 V3 输入仍保持无效；ROS 源龄仅是候选时钟比较，机器人单调时钟不与工作站相减。现场 phase、ready、last_applied_sequence 和 token 都不算 Yichao ACK。尚未验收在线 V3 决策、机载 shadow 或 active；共同坐标、训练 soft limits、历史、时钟转换、几何保护及精确 Controller dirty patch 仍需补证据。
