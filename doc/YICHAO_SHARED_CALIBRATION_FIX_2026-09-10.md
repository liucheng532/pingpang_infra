# Yichao 每次启动读取共享标定（2026-09-10）

最终状态：用户确认动捕故障，结束今天测试、明天再测。本轮工作站输入/Planner及双机Python/原生/记录子进程已全部退出，三机复核无本任务进程、command发布者为空。两套Yichao tmux已保存控制台并移除，共享rosmaster和日志保留。Predictor SDK子进程由既有启动器按超时逻辑最终SIGKILL退出（exit=-9），其他所核对入口以SIGINT退出；没有重启或继续测试。软件退出不等于物理断电确认。改动和标定副本保持，明天先恢复动捕，再进行物理站立和回球验收。停止证据：[离场退出与复核](device_context/yichao_end_of_day_stop_20260910_014920/)。以下输入中断属于停机前诊断。

最新状态：用户询问状态时，两台已由 policy 转为 `waiting_inputs`。只读3秒采样，上游/私有双侧 torso 均0条，私有 state 仍分别130/131条；关节和IMU源龄约4–6ms，torso源龄约116s。Predictor stderr 明确出现 VRPN TCP 连接断开，输入/Planner进程仍存活。Planner最新 IDLE、`missing_private_robot_state`、fault=null、commands=[]，未完成击球。本轮没有因此重启程序或要求再次R2；需要恢复新鲜动捕输入。软件等待不代表断力矩，也不能依据缓存的位姿宣称稳定站立。证据为 `input_interruption_check.json`、`user_status_latest.json`。下文双机policy为较早的启动成功记录。

用户授权修复 Yichao 冻结旧标定的问题，随后明确要求改完启动，并确认双机已重启、进入调试模式且现场安全。本次默认仍是 actor 199＋filter＋V9 i19000，66 镜像 OUTWARD_HOLD、198 HOME_HOLD。

## 修复内容

`tools/run_predictor_snapshot.py` 原来强制将 `PINGPANG_CALIBRATION_CONFIG` 指向 `predictor_snapshot_20260908/calibration_config.json`。虽然部署清单已记录共享 `calibration_source`，启动器没有使用它。

现在每次启动从清单指定的共享路径读取最新文件：

`/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/calib/calibration_config.json`

新增 `src/yichao_v3_v9/predictor_calibration.py`，复用原 Predictor 的标定校验器，要求桌面和双机 tracker 完整、刚体编号不重复。将已校验的原始字节写入 `output/predictor_runs/<session>/calibration_config.json`，Predictor 子进程只加载这份本轮副本。

共享文件缺失或不合法时启动失败，不回退旧快照。每次启动重新取最新共享配置；一次运行中保持同一份配置，避免中途更新产生坐标突变。`calibration_provenance.json` 和 `run.json.calibration` 记录来源、实际副本路径、SHA256、标定时间、桌面矩阵、198/66 刚体编号与外参。启动子进程前即写入来源记录，控制台也输出该信息。

没有改 Fixed 目录、共享标定本身、V9 模型/reference、镜像、R2 或动态平衡循环。机载不需要此次代码更新。

## 验证和同步

- 本地和工作站各 42 项测试通过，覆盖每日更新、新旧会话隔离、缺失/非法配置、实际启动器环境变量及启动前日志等。
- 测试使用真实冻结 Predictor 校验器；启动器测试模拟 ROS 图和子进程，没有连接 SDK 或启动控制。
- 工作站无总线导入真实 `params.py`，确认新刚体编号和桌面矩阵实际生效。矩阵按原代码转换为 float32，与期望 float32 矩阵逐项一致；相对 JSON 双精度最大差 2.284e-8，为类型转换精度。
- 当前共享版本 `2026-09-09T22:52:07+08:00`，198/left→rigid 0，66/right→rigid 1；SHA256 `473780a838b70be5a2198a2be720e74ec24cc6c8a552bed62b4e971965ded8f6`。
- 工作站同步启动器、新模块、新测试、更新后的锁文件共 4 文件，备份在独立 Yichao 根目录 `backups/shared_calibration_fix_20260910_013548/`；同步后远端与本地设备副本 SHA256 一致。旧 Predictor 代码快照及其标定资产保持原样，旧标定不再作为运行来源。
- 启动前双机 SSH 在线，未发现旧控制进程，原生启动器和 V9 Python 入口 SHA256 与本地一致。

证据：[修改、同步和验证目录](device_context/shared_calibration_fix_20260910_013548/)。原问题对照见[部署差异诊断](FIXED_VS_YICHAO_66_DIAGNOSIS_2026-09-10.md)。本次消除了已确认的旧标定接入错误，物理站立恢复仍需本轮实机观察。

## 本轮启动

已实际启动会话 `active_20260910_013920_6a620a`（工作站时钟）。工作站输入 tmux `yichao_active_inputs`，双机控制 tmux `yichao_v9_active`；工作站 owner PID 3764652/start 81228769。66 原生 PID 3483/start 64699、Python PID 3591/start 64788；198 原生 PID 3939/start 62800、Python PID 4051/start 62866，启动后均再次核对。

初始两台均 waiting_r2，已告知用户分两次 R2。随后本轮私有 state 订阅 3 秒各收到 148 条：两台均已进入 `policy`，输入 `sensors_recent=true`；66 OUTWARD_HOLD，198 HOME_HOLD。`policy_ready.json` 确认双机就绪，`relay.log` 已输出 active started=true，Planner command 发布者已注册。不能再要求用户重复按两次 R2，也不能沿用启动初期“Planner 尚未启动”的快照。

实际 Predictor stdout 已确认使用本次副本，版本、SHA256 和 0/1 刚体映射与无总线检查一致。66 首帧 pelvis_y 约 −0.387m，末次采样约 −0.552m；198 末次约 +0.802m。读数变化证明输入已改变，不证明物理站稳或无漂移。尚未收到本轮站立、接球的现场验收结果。

本轮日志在工作站独立根目录下：

- `output/sessions/active_20260910_013920_6a620a/`：输入、姿态、就绪和 Planner `relay.jsonl` 等实际日志。
- `output/predictor_runs/active_20260910_013920_6a620a/`：标定副本、来源记录、Predictor stdout/stderr、球预测 CSV 和源龄日志。
- `output/calibration_corrected_20260910_013906/`：两入口控制台及启动记录。

机载日志在各自独立根目录 `output/active_20260910_013920_6a620a/policy_66/` 或 `policy_198/` 下，metadata 已读取核对。完整状态和进程证据见修改目录中的 `launched.json`、`private_states_verified.json`、`*_started_verified.json`。首次工作站状态诊断误订阅原 `/doubles/.../state`，收到零条；随后订阅本任务私有 namespace 核对成功，不能将第一次零条当成输入故障。
