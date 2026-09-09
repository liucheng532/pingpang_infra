# Yichao 当前可执行的手动启动步骤

2026-09-08 晚间更新：[两台独立制品补齐与工作站配置](YICHAO_PREPARATION_2026-09-08_EVENING.md)。两台 SSH 已恢复，私有环境/接收器/模型导入验证完成；机载 shadow 和真实输入尚未验收，active 仍关闭。当前角色：左侧 198、右侧 66。

最新软件实现及证据见[完整 relay 基准](YICHAO_FULL_RELAY_PROGRESS_2026-09-08.md)：工作站两拍软件回放通过，active 仍关闭。朋友测试期间不运行带 `--relay-session` 的控制客户端，不为本任务操作遥控器。

2026-09-08：朋友将进行真机测试，当前不启动本任务机载程序。独立移动执行接口已有代码，但两台机载准备尚不一致，不能把它作为可直接启动的 active 方案；见[最新进度](YICHAO_ONBOARD_MOVEMENT_PROGRESS_2026-09-08.md)。本页原有命令仍只用于回放/诊断。

2026-09-07：已交付的是独立 V3 回放和诊断 shadow。真实命令发布、执行反馈和完整击球交接尚未完成，`--mode active` 仍被程序阻止。本页不是进入 default 或真机接球的启动卡。

## 1. 登录 Planner 工作站

```bash
ssh odl@172.16.3.126
cd /home/odl/codebase/yichao_v3_v9_e14fd5b
```

本方案使用目录内 `.venv`，不需要 `conda activate`。启动脚本自动使用独立 Python、ROS Python 依赖和 CPU 线程限制。

```bash
./.venv/bin/python -V
```

这一步只核对解释器存在。不要通过安装或升级依赖处理异常，把报错保留即可。

## 2. 启动只读 shadow

```bash
./run_planner.sh --planner yichao --mode shadow --actor 199 --subscriber-ip 172.16.3.126
```

运行 20 秒后自动退出。不会启动 Predictor、机载 policy、g1_control，也不会发送电机或 Planner 控制命令。所用 ROS master 在工作站本机；`subscriber-ip` 是本任务节点的广告地址。

启动打印 `run_dir`，例如 `output/runs/20260907T150718Z_40c5aabc`。`ready_to_start=true` 仅表示这个诊断进程可以启动，不表示机器人/Yichao active 就绪。

## 3. 查看本次日志

```bash
./.venv/bin/python - <<'PY'
from pathlib import Path
root = Path('output/runs')
runs = [p for p in root.iterdir() if p.is_dir() and (p/'run.json').is_file()]
if not runs:
    raise SystemExit('没有运行记录')
p = max(runs, key=lambda x: (x/'run.json').stat().st_mtime)
print('日志目录:', p)
for name in ('run.json', 'stderr.log', 'stdout.log'):
    f = p/name
    if f.exists():
        print('\n'+name+'\n'+f.read_text()[-12000:])
PY
```

需要的真实输入为双机 `/doubles/table_left/state`、`/doubles/table_right/state`，以及原双打 ball prediction 或现有 Predictor monitor。上一轮 monitor 有 583 次消息，两路 state 均缺失，真实决策为 0。只看到进程正常退出不等于模型收到完整真实输入。

## 4. 可选：独立离线回放

以下命令使用合成夹具；用于检查模型、安全过滤和模拟 scheduler，不是实机验证。

```bash
./run_planner.sh --planner yichao --mode replay --actor 199 --input fixtures/two_shots.jsonl
```

每次生成新的日志目录。上述两个模式都不需要操作遥控器。

## 遥控器与 default 的边界

原 V9 active runner 的提示是：先 `Press R2 to calibrate`，按一次后进入预设校准姿态；随后 `Press R2 to start controller`，再按一次才进入底层 policy。第二次不是“确认校准”而是启动控制。运行中再次 R2 可能重新向 nominal 姿态运动，不是断力矩急停。

目前的 Yichao shadow 不会出现上述两段提示，不应据此按 R2。`g1_control` 本身还会在 Python 校准前自动执行约 3 秒零关节姿态插值；不能用“还没按 R2”推断无电机输出。

用户最新现场状态为机器人落地、有绳索保护，已授权推进 default 操作；未执行本任务电机程序。现场具体急停按钮/开关和操作人尚待明确。Yichao active 的代码集成及现场验收仍须完成，不能把原 Fixed Planner 的启动命令标成 Yichao。

## 已核对的网络差异

198 当前无法访问 `192.168.123.165:11311`，两台均可访问 `172.16.3.126:11311`。198 原启动脚本硬编码 `172.16.4.184`，两台本轮访问该端口均超时。尚未改动原脚本或共享网络。

独立诊断位姿转发已实测：通过工作站管理地址和 `/yichao_v3_v9/pose_route_152248/...` 专用 topic，198/66 分别收到 822/830 条 torso，转发无解析拒绝/队列丢弃。此 30 秒转发已结束，topic 不是当前持续服务；源时钟仍未对齐，不能当作正式 active 输入。

证据：`doc/yichao_v3_v9/robot_start_preflight/`。完整实施进度见 `YICHAO_V3_V9_RUNTIME_SHADOW_2026-09-07.md`。
