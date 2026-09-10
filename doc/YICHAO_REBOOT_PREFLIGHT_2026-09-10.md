# 9 月 10 日下午重启后的测试准备

15:41–15:44 完成工作站、66 和 198 的连接、入口及无总线加载检查。当前共享标定已被实际 Predictor 加载；生产代码已有每日标定适配，本轮没有再次修改 policy、reference motion 或硬编码刚体编号。

用户提供的逸超私聊已明确：口头 V4 就是 `20260906_v3_planner_190_199`。本次继续 actor 199＋safe_filter_v3＋V9 i19000，默认 66 镜像 OUTWARD_HOLD、198 HOME_HOLD。

## 本次验证

- 工作站及双机 SSH 可用；工作站经既有密钥、有线地址访问双机的正式启动预检通过。双机未发现控制进程。
- 两台 `run_active_control.py --check`、`run_onboard_active.py --check` 通过；双机控制入口 SHA256 与本地一致。没有启动原生驱动或 policy。
- 工作站 Predictor/每日标定读取/active 启动入口与本地一致。工作站资产锁与其本地设备副本一致；相比开发环境仅 requirements.lock 哈希不同，保留部署环境版本。
- 当前标定来源为 `/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/calib/calibration_config.json`，更新时间 `2026-09-10T15:21:24+08:00`，SHA256 `1b81101f4008447b482c00a66f6db989c60de56866e026856ce07d46ef2d0cc1`。198/table_left→rigid 1，66/table_right→rigid 0。实际 params 的编号、桌面矩阵与双方平移/四元数均和本轮副本核对通过。
- 输入专用会话 `reboot_inputs_20260910_154405` 限时运行 Predictor 12 秒：收到左侧 3068、右侧 3074 条姿态；有效接收时段约 11.05 秒，最大接收间隔分别 13.69/11.37 ms。run.json 确认使用上述新标定。
- 本轮 Predictor 已退出：到时后既有退出逻辑依次 SIGINT、SIGTERM、SIGKILL，子进程 exit=-9；不能称 SDK 自然退出。输入/command 发布者均已撤回。共享 ROS master 上仍有历史私有节点注册，不代表旧机器人进程存活，未改共享 master。

## 尚待确认

末帧 torso 的 mocap_origin 坐标：198 约 `(1.178, 1.080, 0.817)` m，66 约 `(1.163, 0.184, 0.807)` m。两台横向坐标同为正，已询问用户是否尚未摆到正式接球位置。不能据此直接改坐标原点、对调刚体，也不能把数据连续视为物理站稳或训练坐标已验收。

本轮仅完成测试准备和限时输入检查，没有启动电机或恢复昨晚事务。

## 正式测试入口

两终端均在工作站 `odl@172.16.3.126`。第一终端：

```bash
cd /home/odl/codebase/yichao_v3_v9_e14fd5b
python3 -B scripts/run_yichao_stack.py start active
```

第一终端显示 Terminal 1 ready 后，第二终端：

```bash
cd /home/odl/codebase/yichao_v3_v9_e14fd5b
bash scripts/start_yichao_dual_robots.sh start active
```

启动器会再次检查双机占用，创建新 session，并重新读取当时共享标定。不要运行历史证据目录中硬编码昨晚标定哈希/编号的 `launch_workstation.py`。实际启动后先查看新会话记录，再由现场完成每台第一次 R2 校准及第二次 R2 policy；运行中 R2 是重新校准，不是断力矩。

证据：`doc/device_context/reboot_preflight_20260910/`。`workstation_preflight.json` 中双机检查已通过，随后诊断脚本本身的快照路径重复导致失败；独立 `calibration_preflight.json` 修正读取路径后通过，不是生产启动器故障。首次 198 清单因机载没有 tmux 命令未输出，后续 `left_inventory_verified.json` 完成复核；正式 tmux 位于工作站，机载无需安装。
