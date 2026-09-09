# Yichao V3 / V9 本地仿真

本目录只用于笔记本 Isaac Lab 仿真，不包含 ROS/DDS/LCM 控制入口。
现有实机程序保持原状；这里的源码、环境和运行结果无需部署到三台现场设备。

## 环境

`yichao_sim2sim` 从本机 `env_isaaclab` 克隆：Python 3.11、Isaac Sim 5.0、
Isaac Lab 2.2.1（Python package 0.46.3）、PyTorch 2.7.0+cu128。
在克隆环境内安装 rsl-rl-lib 5.0.1、TensorDict 0.12.2、onnxscript 0.5.4、pyvers 0.2.0。
CUDA 张量计算、`doubles_planner.safe_rl` 导入已实测通过。无需手动安装。

完整包清单见 `environment.freeze.txt` 和 `environment.conda-explicit.txt`。
Isaac Lab 的 editable 源码仍来自 `/home/lyz/Desktop/code/robo_dance/IsaacLab`，
这里只读复用；不要移动该目录。`pip check` 有原环境继承的 sentry-sdk 版本告警，
见 `logs/pip_check.log`，并非本次 runner 安装新增的冲突。

## 运行

从 PingPong 根目录执行，两种方式均调用独立 Conda 环境，不必先 activate：

```bash
# 快速双拍，无画面
./local_sim2sim/run.sh --headless --disable-video --steps 4 --shots 2

# 24 拍并保存带状态标注的视频
./local_sim2sim/run.sh --headless

# 打开 Isaac 窗口，有限拍次运行
./local_sim2sim/run.sh --steps 8 --shots 6
```

首次渲染需要初始化 shader，启动可能持续数分钟。
输出目录默认 `local_sim2sim/outputs/<时间>`，也可传 `--output-dir <独立目录>`。
每次使用新目录，避免覆盖已有证据。评估脚本退出码 0 只说明运行完成；
实际结果用下列命令查看：

```bash
python3 local_sim2sim/check_result.py local_sim2sim/outputs/<时间>
```

`assessment.json` 分别记录链路检查、质量失败次数和精确训练补丁是否到位。
链路检查包含完成指定拍数、双机交替、交接、有限误差、无碰撞/终止/超时/距离越界。
击球质量采用现有 evaluator 阈值：位置 0.08 m、速度 1 m/s、朝向 0.10 rad、时间 0.04 s。

## 固定资产与证据边界

- `planner/`：`pingpang_planner@e14fd5b` 的独立 detached worktree。
- `controller/`：`pingpang_controller@47359de` 的独立 detached worktree，V9 timed-handoff。
- actor 199 + frozen safe_filter_v3 + V9 student i19000，使用交付原文件。
- `assets/`：用户指定 `unitree_description.zip` 的 G1 URDF/mesh 副本；
  原始 URDF SHA256 `2efe5bddebd5a3ba4685a085031892ed14abc16b343970876380ae61a08d76cc`。
- motion bank 复用项目 `pingpang_assets/0718-move-160-80hz` 和 `0302_combined`。
- `run.py` 只重定向模型资产位置和组装命令；保留原 evaluator 的版本/模型检查。

当前 Controller 是交付匹配的 Git 基础版本。训练时未提交的
`distill_commands.py`（目标 SHA256 `835a9c69…`）尚未找到。
每次运行的 `local_manifest.json` 明确标记这一差异，不声称逐字节复现 formal run。
没有修改或放宽原始版本校验，没有换用 V10/V11。

这是同一 Isaac 引擎中的本地闭环复评：机器人由 PhysX 驱动，模型实际推理，
来球由 evaluator 生成，成功指标是击球目标跟踪。**没有模拟球拍接触后的回球飞行，
也尚未完成 Isaac → MuJoCo 的跨引擎迁移**。它不构成实机接入验收。
当前运行的是交付自带 Planner/Controller 仿真链路，并未经过
`yichao_v3_v9/` 的实机通信适配层。视频中的 left/right 沿用训练环境命名，
不能直接解释成实机 198/66 的坐标或角色映射。

运行日志位于 `logs/`，每次摘要、manifest、assessment、视频位于对应输出目录。

## 2026-09-08 实测

24 拍及 24 次严格交替交接完成，物理碰撞、终止、同时 HIT、超时均为 0。
质量仅 4/24 拍全部达标，平均位置误差 12.9 cm，尚不能推进实机。
[视频](outputs/actor199_24shots/v0_safe_rl_rollout.mp4)与
[完整报告](../doc/YICHAO_LOCAL_SIM2SIM_2026-09-08.md)已保存。

首次视频运行在输出写完后卡在 Kit 卸载，已仅清理该本地进程。
启动器现在等 evaluator 关闭环境、保存结果后写入 `run_completion.json`，
再退出解释器，避免 Kit 最后的卸载挂起；异常返回非零。
该修正已通过双拍复测，进程退出码 0，计算结果与首轮双拍一致。
