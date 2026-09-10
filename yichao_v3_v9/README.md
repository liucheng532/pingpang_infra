# Yichao Planner 适配（复用 Fixed）

本项目只替换**工作站二的上层 Planner**。原 Fixed Planner、Predictor、66/198
Controller 的源码、模型和启动入口保持原样。目录名 `yichao_v3_v9` 是历史名称，
不代表本适配器会安装或启动 V9 Controller。

```text
现有 Predictor ── ball_prediction ──► Yichao Planner（工作站二）
现有 Controller 66/198 ── state ────►       │
现有 Controller 66/198 ◄── command ─────────┘
                     Monitor 只观察同一套接口
```

## 实现边界

- `baseline/doubles_planner/`：当前 Fixed 工作树的独立副本，原字节保留。
  复用状态解析、击球归属、reservation/commit、阶段反馈及 RETURN 重试。
- `src/yichao_v3_v9/inputs.py`：从既有 state/ball 构造 36D actor、85D filter 输入。
  66 为训练负 Y 槽，198 为正 Y 槽。state 已是物理 LAB 关节及 pelvis，
  不再次镜像、FK 或应用桌面标定。历史用共同的 20 ms 接收时间网格；这是接收时间近似，
  不是声称已经同步两台机器人的传感器时钟。
- `inference.py`：actor 199 + 原 safe_filter_v3，保留交付模型。
  候选集合在既有 Fixed 工作区内选择，再使用原过滤器评分；不截断一个已选好的目标后直接下发。
- `runtime.py`：Yichao 位置策略接入 Fixed 生命周期。2026-09-11 已删除适配层新增的
  双机距模型目标均不超过 6 cm 才能 HIT 的条件。有效模型决策下，HIT 继续按原
  Fixed 时间窗和阶段规则提交；等待提交时仍使用原 `stage` 接口下发横移目标。
  现有 Controller 在 HIT 内清零 target_base；本次仅取消到位门槛，角色分配、
  每球目标缓存及 Controller 行为未改。
- `monitor/`：复用 Fixed 页面/订阅，增加推理状态、录制、回放、下载/导入。
- `baseline/controller_reference/`、`predictor_reference/`、`training_reference/`：
  **只读测试参考，不进入部署包，也没有新版 Controller/Predictor 启动入口**。

Controller 协议中的 `planner_mode` 是接收分支选择：HOLD/RETURN 使用其已有
`fixed_relay` 分支，STAGE 使用已有通用位置分支。`position_strategy` 和
`/doubles/yichao/status` 标识真实的 Yichao 模型；这不启动原 Fixed Planner。

## 两个便捷入口（推荐）

在工作站二打开两个终端，均进入 `/home/odl/codebase/yichao_planner_fixed_adapter`。
确认没有重复运行的 Fixed/Yichao Planner 或 Predictor 后：

```bash
# 终端一：原 Predictor + 新 Yichao Planner（active）+ 新 Monitor
bash scripts/start_yichao_workstation.sh

# 终端二：调用原 Fixed V11 双机入口，等待各自两步 R2
bash scripts/start_yichao_robots.sh
```

这两个入口参考 Fixed 的工作站/双机划分。工作站入口复用 Fixed 的组件监督与退出逻辑，
Predictor 路径、Python 环境和共享标定配置沿用 Fixed；双机入口不复制或修改 Controller，
直接调用原 `start_v11_dual_robots.sh start active normal active 1.0 v11-teacher`。
原入口的占用检查、模型预检和 tmux 控制台保持生效。

新版 Monitor 为 **http://172.16.3.126:8090**，与旧 Yichao 8089 分开。
工作站程序前台运行，日志目录会打印到终端；Ctrl-C 会停止本入口创建的 Predictor、
Planner 和 Monitor，不会停止双机 Controller。机器人停止仍使用原现场停机流程；
包装入口 `bash scripts/start_yichao_robots.sh stop` 只转交原脚本的 stop。

```bash
# 只显示命令，不启动 ROS/机器人，不连接 SSH
bash scripts/start_yichao_workstation.sh dry-run
bash scripts/start_yichao_robots.sh dry-run

# 双机原预检、状态、控制台
bash scripts/start_yichao_robots.sh check
bash scripts/start_yichao_robots.sh status
tmux attach -t v11_dual_robots
```

每台在 `About to calibrate` 后按下并松开第一次 R2，等待默认姿态到位及
`Starting pose calibrated`，再按下并松开第二次 R2 进入动态 Controller。
双机实际待命正常、位置和状态正确更新、能识别实测球后再发球。
`bash scripts/start_yichao_workstation.sh shadow` 不下发 Planner 命令；它不改变
双机脚本的 active 模式，也不是机器人断力矩模式。

## 分开运行（已有 Predictor 时）

先使用原有方式准备 Predictor 与两台 Controller。切换上层 Planner 时，
原 Fixed Planner 必须已退出；本适配器只检查冲突，不自动停止其他程序。

```bash
# 仅显示启动清单，没有 ROS/设备副作用
.venv/bin/python scripts/run_yichao_stack.py

# 只读推理：不创建机器人 command 发布者
.venv/bin/python scripts/run_yichao_stack.py --start

# 正式上层命令发布（须在设备控制窗口中执行）
.venv/bin/python scripts/run_yichao_stack.py --start --active
```

这些入口只启动 Yichao Planner 和 Monitor。已有 Monitor 时可加
`--without-monitor`。退出仅回收本入口创建的进程组，**不会退出机器人 Controller
或 Predictor，也不代表断力矩**。机器人启停继续使用其原有操作流程。

每次运行保存 `output/sessions/fixed_时间_随机串/`：组件 stdout、PID/starttime、
输入、36D/85D、actor/filter、command/state、退出码及日志丢弃/写错统计。
日志异步分段，保留全部片段。标定仍由现有 Predictor 加载共享配置，本适配器
不会因读取磁盘文件就声称 Predictor 已加载该版本。

Monitor：`http://工作站二:8089`。单独只读运行：

```bash
PYTHONPATH=src .venv/bin/python -m yichao_v3_v9.monitor.server
```

默认无 ROS 设置发布者；`--enable-config` 才开放 Fixed 原有参数配置。
浏览器回放禁止提交运行参数，服务器录制不随浏览器关闭停止。

## 验证与尚未证明的事项

使用真实模型做了 ONNX/TorchScript、冻结 filter、特征向量对照；使用原 Controller
实际 observe/scheduler/动作资产验证 STAGE 目标；六拍协议回放验证交替和 RETURN。
测试中机器人位置反馈是预设数据，**没有模拟物理平衡**。2026-09-10 场地重建并加载22:25:51共享标定后，
用户现场确认两台站稳，实际来球时会来回移动换位并尝试接球。
这已验证实机行为触发，但未统计成功回球率或完成长期稳定性验收。
若 Predictor 没有有效球，Planner 仍不能接球；重写不会把缺失输入补成有效球。

旧自定义 runner、私有协议、恢复/启动器和旧部署配置已从工作源码删除。
用户要求清理的重写前归档及旧 V9 资产已列入本次 GitHub 发布后的删除清单；
删除结果记录在 `../doc/yichao_fixed_rebuild_20260910/cleanup.json`。
当前模型、只读依赖参考和现场日志保留。

## GitHub 源码范围与外部依赖

GitHub 保存本地适配、Fixed 核心副本、测试与启动脚本，不包含模型权重、虚拟环境、
现场日志、设备镜像及 Controller/Predictor/training 的完整参考快照。
`baseline/SOURCE_MANIFEST.json` 保留 Fixed 来源与哈希。旧版源码仍可从 Git 历史查看。

工作站现有部署可直接使用两个启动入口；新克隆不是完整运行环境，需要补齐：

- 原 Fixed Predictor、双机 V11 Controller 及各自 Python/ROS 环境，路径见启动脚本。
- `vendor/handoff/20260906_v3_planner_190_199/onnx/` 的 actor190、actor199、safe_filter_v3。
- `vendor/training_assets/` 的 URDF 与关节归一化 profile，格式/哈希由 joint_normalization.py 校验。
- 完整离线模型/Controller 回放测试还需要 `baseline/controller_reference/`、
  `baseline/training_reference/` 及交付 TorchScript 模型；这些不是旧部署备份，仍在本地保留。

仅有源码时，可使用提供的环境和依赖运行运维测试；完整套件要求上述资产齐全。
