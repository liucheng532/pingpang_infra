# Yichao 部署前残留清理（2026-09-08）

用户授权：清理残留，准备启动 Yichao。已完成指定旧进程、已退出 tmux 会话和失效 ROS 注册的清理，未启动新控制程序，未修改原 Fixed/V11 源码、环境、标定或日志。

## 实际操作及结果

| 对象 | 清理前证据 | 操作与复核 |
| --- | --- | --- |
| 工作站旧启动器 2602490 | `scripts/run_doubles_stack.py --shadow`，状态 T，starttime `71380977`，cwd 为原 Runtime | 确认所有子进程均为 Z 后，以 pidfd 固定进程身份发送 TERM、CONT，使其既有退出处理器执行；进程正常消失 |
| 子进程 2602492/2602493/2602494 | 均为上述父进程的僵尸子进程 | 随父进程退出回收；复查均不存在 |
| tmux `v11_dual_robots` | 4 个 pane 全部 dead | 保存最近窗口输出，再次确认列表不变后移除该会话；复查该 tmux server 已无会话 |
| 失效 ROS 节点 | 19 个旧 recorder、2 个旧 monitor、2 个机器人 policy | 每个节点两次 XMLRPC 连接均被拒绝且 URI 未变化，逐项注销其 publisher/subscriber/service；合计 23 个节点 |
| rosmaster | PID 1150539，starttime `12294544` | 保留，身份不变；清理后 getSystemState 为三组空列表 |

初次尝试因远端系统 Python 3.8 没有 `os.pidfd_open` 而在发送信号前退出；随后通过 Linux libc pidfd 系统调用完成。未因此使用宽泛 pkill 或修改共享 Python 环境。

## 清理后启动预检

- 笔记本可 SSH 到工作站、66 和 198；三台的控制进程扫描均为空。
- 工作站使用新双入口现有 SSH 参数（既有 `pingpang` 密钥、严格 known_hosts、66 `192.168.123.164`、198 `192.168.124.164`）执行只读预检，两台均通过，无现有控制器。
- 工作站当时 24 CPU，load average 约 0.49/0.46/0.45，可用磁盘约 745 GiB。这里只作资源快照，不等于实时性验收。
- 没有执行新 Predictor、机载 shadow、g1_control、policy 或 active。已移除残留引起的冲突；完整双入口组合运行、真实输入及闭环验收仍待执行。
- 任务版本保持交付 `20260906_v3_planner_190_199`，actor 199＋安全过滤器＋配套 V9 i19000。当前双入口仅支持 shadow，不能把清理完成理解为 active 已可运行。

## 证据

- [工作站清理前后及每项注销结果](yichao_v3_v9/cleanup_20260908T154931Z/report.json)
- [工作站到双机启动预检](yichao_v3_v9/cleanup_20260908T154931Z/launch_preflight.json)
- 同目录 `tmux_pane_*.txt` 保存被移除会话的退出窗口内容。
- 工作站原始记录目录：`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/cleanup_20260908T154931Z`。
- [三设备连接复查目录](yichao_v3_v9/connectivity_20260908/)含本轮清理前后快照。各设备时间原样保留，不据此假定跨设备时钟已经对齐。

未删除旧运行日志、源码、模型或环境。没有待同步的业务代码修改。
