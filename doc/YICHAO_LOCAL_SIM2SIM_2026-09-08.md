# Yichao 本地 Isaac 仿真记录（2026-09-08）

## 当前结论

笔记本环境已配置，actor 199＋safe_filter_v3＋V9 i19000 已实际驱动 Isaac
双机器人完成 24 拍、24 次交接。没有物理碰撞、机器人终止、同时 HIT、超时或
三项距离阈值越界。**本地运行链路通过，击球质量未通过，实机接入继续暂停。**

这次没有运行、停止、修改或同步三台现场设备上的程序；新增内容限于笔记本
`local_sim2sim/` 和本地文档。没有改动论文仓库、原 Planner/Controller 工作树，
也没有修改独立 `yichao_v3_v9` 实机适配源码，因此无对应远端部署文件需要同步。

## 实测环境与配置

- GPU：RTX 3080 Ti Laptop，16 GB；RAM 31 GiB；驱动 580.173.02。
- Conda：`/home/lyz/miniconda3/envs/yichao_sim2sim`，从 `env_isaaclab` 克隆。
- Python 3.11；Isaac Sim 5.0.0.0；Isaac Lab 2.2.1 / package 0.46.3。
- PyTorch 2.7.0+cu128；rsl-rl-lib 5.0.1；TensorDict 0.12.2；onnxscript 0.5.4；pyvers 0.2.0。
- 已通过 CUDA 张量计算、V3 runner 导入和实际双机物理运行。无需用户手动安装。
- 完整环境和安装日志保存在 `local_sim2sim/environment.*`、`local_sim2sim/logs/`。
- rsl-rl 版本按交付环境固定；该版本安装包见 [官方 PyPI 5.0.1](https://pypi.org/project/rsl-rl-lib/5.0.1/)。
- 原环境已存在的 sentry-sdk 元数据冲突仍记录于 `logs/pip_check.log`；未阻止运行。

Planner 独立 worktree 为 `e14fd5b`，Controller 为 `47359de`，后者通过原始 V9
模型/contract 检查。没有改变版本检查、模型参数、物理控制和安全阈值。
机器人使用用户指定压缩包内的 G1 URDF/mesh，36 处 mesh 引用全部可解析。
URDF SHA256：`2efe5bddebd5a3ba4685a085031892ed14abc16b343970876380ae61a08d76cc`。
V9 student SHA256：`2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a`。

动作库复用 `pingpang_assets/0718-move-160-80hz` 和 `0302_combined`。
评估参数按交付 DEPLOYMENT：seed 10000，24 shots，最多 64 steps，low-level steps 8，
warmup 150，shot interval 1.2 s，ball speed scale 1.25，target Y [-0.9, 0.9]，
candidate radius 0.75、7×7 grid，hard-guard，filter 开启。

## 结果

| 指标 | 本机 24 拍 | 交付中 RL+CBF 24 拍 |
| --- | ---: | ---: |
| 实测击球事件 | 24 | 24 |
| 交接 | 24 | 24 |
| 物理碰撞 | 0 | 0 |
| 机器人终止 | 0 | 0 |
| 同时 HIT | 0 | 0 |
| 三项距离任一越界 | 0 | 0 |
| 全部击球质量阈值通过 | 4/24 | 13/24 |
| 平均击球位置误差 | 0.1291 m | 0.0827 m |
| 最大击球位置误差 | 0.3773 m | 0.1789 m |
| 平均拍速误差 | 0.6082 m/s | 0.3006 m/s |
| 平均朝向误差 | 0.0992 rad | 0.0950 rad |
| 平均时间误差 | 0.00989 s | 0.00946 s |

本机最小 base / inner-hand / inner-feet 距离分别为 0.7046 / 0.3525 / 0.3181 m。
完整 mesh 几何风险有 3 个规划步报告，但实际 PhysX 接触碰撞为 0；两者不可混同。
actor 本轮没有触发 learned-filter 干预，不构成过滤器干预有效性的压力测试。
质量阈值保持源码默认：位置 0.08 m、速度 1 m/s、朝向 0.10 rad、时间 0.04 s。
上述历史对照有源码/资产差异，不能当作严格成对复现或因果归因。

证据：

- [24 拍结果](../local_sim2sim/outputs/actor199_24shots/v0_safe_rl_rollout.json)
- [独立结果判定](../local_sim2sim/outputs/actor199_24shots/assessment.json)
- [24 拍视频](../local_sim2sim/outputs/actor199_24shots/v0_safe_rl_rollout.mp4)
- [运行文件哈希及参数](../local_sim2sim/outputs/actor199_24shots/local_manifest.json)
- [首轮双拍结果](../local_sim2sim/outputs/baseline_smoke/v0_safe_rl_rollout.json)
- [原交付统计](../pingpang_planner/handoff/20260906_v3_planner_190_199/validation/v3_metrics.json)

视频经 ffprobe 检查：960×540、25 FPS、767 帧、30.68 秒；抽帧确认双机器人、球台、
HIT/OUTWARD/HOME 状态可见。原 HUD 小字号粗体较拥挤，精确数值以 JSON 为准。
原 evaluator 检查了帧与状态对齐，768 个采样记录按编码后的 767 帧去掉一个末尾记录。

## 退出处理

24 拍在 20:36:11 完成并保存结果后，Isaac Kit 插件卸载挂起。先确认结果、视频
可读取以及确切 PID 282931；SIGTERM 无效后只对该本任务进程使用 SIGKILL。
因此首次视频命令的最终 shell 退出码是 137，不能记成整个进程正常退出；
这不改变已完整保存的 24 拍物理结果。

本地 `run.py` 已在 evaluator 返回、环境关闭和文件写完后记录 `run_completion.json`，
刷新标准输出，再退出解释器，避开 Kit 最后卸载阶段。异常仍打印并返回非零退出码。
短复测记录存于 `outputs/exit_fix_smoke/`、`logs/exit_fix_smoke.log`。
复测已完成两拍及两次交接，进程退出码 0；metrics、击球误差统计、最小距离和
交替序列均与首轮无画面双拍逐项相同。进程已退出，GPU 资源已释放。

## 尚未验收的部分

1. 训练时未提交的 `distill_commands.py` 补丁（SHA256 `835a9c69…`）尚未取得。
   当前基础代码为 `25dce7ef…`，每轮 manifest 明确 `exact_training_patch_available=false`。
   没有伪造补丁或放宽校验。
2. 本地动作库虽已结构检查，仍未证明与训练端逐字节相同；指定 URDF 的来源已固定，
   但精确训练时生成资产与软件栈也尚未全部对齐。质量差异原因需进一步核查。
3. 这是交付原生 Isaac 仿真链路，尚未测试 `yichao_v3_v9` 实机通信层，也不是
   Isaac→MuJoCo 跨引擎验证。仿真 left/right 命名不直接等于 198/66 现场映射。
4. 来球为脚本模型，机器人为物理闭环；尚未模拟球拍接触后的出球飞行/连续对拉。
5. 24 拍质量未达标，不能据此恢复 active 或宣布 sim2real 验收通过。

## 使用

从项目根目录执行：

```bash
./local_sim2sim/run.sh --headless --disable-video --steps 4 --shots 2
./local_sim2sim/run.sh --headless
./local_sim2sim/run.sh --steps 8 --shots 6
```

前两条分别为双拍快速检查、24 拍视频；第三条打开 Isaac 窗口。
完整说明见 [本地仿真 README](../local_sim2sim/README.md)。不需激活 Conda，脚本直接使用独立环境。
