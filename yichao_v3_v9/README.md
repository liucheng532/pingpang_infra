# Yichao V3 → V9 离线接口验证包

工作站副本已落地 `/home/odl/codebase/yichao_v3_v9_e14fd5b`，有独立 `.venv` 与可运行入口，见 [工作站启动说明](WORKSTATION_START.md)。

本目录已经跑通 **36D 特征 → actor 199 → 85D 安全筛选 → 保守 relay → 原版 V9 scheduler/mirror** 的文件回放。190 保留作回归对照。独立的 `run_workstation_observer.sh` 提供 ROS 输入检查和只读诊断，见工作站说明。离线回放入口保持文件输入输出；包内没有控制发布或 active 开关。

这是首个离线验证交付，**不是完整 V3 真机适配验收**。relay 已接入 prepare、成对 peer-clear/HIT、V3 动态 outward 投影、显式 RETURN 与双机返回完成确认；训练端 locomotion preemption、几何保护及精确 dirty patch 仍未验收，见 [lifecycle.json](config/lifecycle.json) 和 [验收报告](../doc/yichao_v3_v9/IMPLEMENTATION_REPORT.md)。代码明确拒绝所有 `provenance=real` 的记录，修改配置中的标志也不能开启真实输入。

## 本地运行

从项目根目录执行；输出文件必须不存在：

```bash
./yichao_v3_v9/run_offline.sh \
  --input yichao_v3_v9/fixtures/two_shots.jsonl \
  --output yichao_v3_v9/output/my_review.jsonl
```

追加 `--actor 190` 可对照另一个 checkpoint。输出 JSONL 包含源记录、36D/85D 特征、nominal/filtered/projected 目标、风险、候选索引、命令、模拟 ACK、完成反馈和耗时；汇总保存在同名 `.summary.json`。

`two_shots.jsonl` 是**合成夹具**：人为给定的骨盆跟随轨迹，速度上限 1.5 m/s，滚动 0.5 s TTS。它用于让接口经历两拍生命周期，不代表真实球轨迹、低层闭环或仿真成功。scheduler 消费真实动作库，但骨盆不会由 student 输出驱动。V9 student 单独验证文件身份和 `1666 → 29` ABI。

重跑验收：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  yichao_v3_v9/.venv/bin/python yichao_v3_v9/tools/test.py
```

测试使用 CPU、单线程和原始交付模型。不会调用采集工具或访问现场。默认测试报告写入本目录 `output/test_results.json`；本地汇总另保存在 `doc/yichao_v3_v9/test_results.json`；历史 CLI 回放证据位于 `output/`。

## 目录和冻结方式

| 路径 | 内容 |
| --- | --- |
| `src/yichao_v3_v9/inputs.py` | 复用冻结 `RealInputAdapter` 的基础有效性约束，加严格时间/历史/单位/槽位检查，再组装 36D/85D |
| `inference.py`（同目录） | CPU ONNX、51 个候选、制品阈值、0.45 m 解析保护；无安全候选锁存故障 |
| `relay.py`（同目录） | prepare、peer-clear/HIT、RETURN 完成状态机；首个确认目标对锁存到 actor 记忆 |
| `protocol.py`（同目录） | 复用冻结 V3 纯投影方法，计算动态 outward、runway 和下一击球人 RETURN 目标 |
| `executor.py`（同目录） | 原版 V9 scheduler 的纯计算包装；完整预校验、离线批次预演、去重和模拟完成反馈 |
| `telemetry.py`、`observer.py`（同目录） | 五个输入 topic 的严格诊断解析、ROS 发布者预检、有界只读接收；所有真实输入仍未验收 |
| `config/interface.json` | 36+85 个逐索引字段说明，以及未通过的真实输入证据项 |
| `config/command.schema.json`、`config/lifecycle.json` | 独立命令及反馈契约、时钟约定、验收边界 |
| `config/assets.lock.json` | 源码、模型、配置、依赖版本文件和主动作副本的 SHA256 |
| `vendor/` | Planner `e14fd5b` 与 deploy `347890e` 的固定 Git 快照，未修改源码 |
| `assets/` | 主动作副本、两机 V9 ONNX/sidecar/关键源码证据、全库核对副本 |
| `fixtures/reference_vectors.json` | 由冻结 V3 特征函数产生的合成参考向量 |
| `requirements.lock`、`.venv/` | 独立复制的 CPU 推理依赖，不修改原有环境 |

启动会验证锁定文件、交付 SHA256SUMS、V9 sidecar/模型/动作索引及已安装依赖版本。运行时不刷新清单、不下载模型、不回退到现场目录。开发修改经过审阅后，可显式执行 `tools/freeze.py` 生成新的本地发布清单；这不是签名或现场验收。

本地 `.venv` 为 Python 3.11、NumPy 1.26.0、ORT 1.23.2、Torch 2.9.1+cpu；工作站的独立 `.venv` 为 Python 3.10、NumPy 2.2.6、ORT 1.23.2、Torch 2.7.0+cpu。两处均通过测试，确切依赖分别保存在各自 `requirements.lock`。依赖从原环境只读复制，未修改共享 site-packages；venv 仍使用对应基础 Python 的标准库。原版 V9 加载器不兼容 NumPy 2.4，未为此修改 V9 源码。

## 反馈与故障

- `accepted` 仅表示 scheduler 在离线副本中受理并锁存。`completed` 另需目标误差 ≤0.06 m、夹具平面速度 ≤0.15 m/s、HOLD phase 持续 ≥0.10 s。
- 所有反馈标记 `simulated_scheduler`。其他控制器的 phase 或 `last_applied_sequence` 不能替代该命令的 ACK。
- 会话须显式建立；旧会话、旧序号、重复 token、同 shot 第二次 HIT、非法字段、过期请求会被拒绝。相同请求重试只返回原受理记录，不重复调用 HIT。
- 离线批次先在 scheduler 状态副本中预演，全部成功才提交。本机制不表示两台真实机器人具备原子提交能力。
- 无效输入、无安全候选或超时后进入 FAULT，不再发新命令。不会自动恢复或假装停止已开始的运动。不存在用 `hold` 物理停车的实现。
- 文件入口拒绝非普通文件，禁止 Python 层网络连接、绑定、发送及子进程启动。底层库限定 CPU；当前入口无控制传输代码。输出排他创建，避免覆盖既有日志。

现场正在运行他人测试。本交付没有更改现场代码、配置、依赖或运行进程。`tools/collect_remote.py` 和 `tools/audit_motion_copies.py` 是此前只读证据采集工具，不属于离线启动链，也不会由测试自动执行。
