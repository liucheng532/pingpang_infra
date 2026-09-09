# Yichao原版重启与停止（2026-09-10）

## 最终状态

用户报告66在两次R2后初始动作不对，先提出试第二种方案，随后要求先kill当前程序。已停止本轮Yichao工作站及双机控制，没有启动HOME_HOLD候选。具体动作异常尚未定位。

- 最终会话：`active_20260910_003226_53c154`。
- 工作站：Planner PID3721369、启动器3719653及Predictor/姿态输入退出。
- 66：原生7094、Python7199及记录子进程退出。
- 198：原生3609、Python3714及记录子进程退出。
- 信号前核对PID、starttime、完整argv与本任务目录；退出后重新枚举三机进程，无本任务控制残留，未额外发SIGTERM/SIGKILL。
- ROS command发布者为空；保存窗口输出后移除`yichao_active_inputs`和`yichao_v9_active`，共享rosmaster PID1150539/start12294544保持。
- 软件退出不代表已核实物理断力矩。日志文件保留，不宣称全部日志完成摘要已验证；Planner退出控制台出现KeyboardInterrupt，包括注销订阅时再次中断。

证据：[进程核对与退出](device_context/yichao_stop_before_candidate_20260910_003756/stopped.json)、[最终核对、控制台和日志](device_context/yichao_stop_before_candidate_20260910_003756/final_verified.json)。

## 本轮启动与重启

用户要求启动Yichao；发现新运行的Fixed active后，用户明确授权kill Fixed。已对确切启动器PID3654220发SIGINT，其Predictor、Monitor、Fixed relay及子进程退出，保留原方案源码、日志及共享rosmaster。双机无总线模型/驱动检查通过。

首次启动会话`active_20260910_002603_41bb29`使用actor199＋filter＋V9 i19000，66原版镜像OUTWARD_HOLD，198 HOME_HOLD。启动后出现66姿态输入间断、198重启，尚未双机就绪；后续只读确认上游和私有姿态恢复、198有线口在线但旧控制进程已消失。用户随后明确要求重启。

重启前退出旧工作站/66进程并确认198无控制残留，回收旧tmux。新会话`active_20260910_003226_53c154`实际启动两台原生驱动和Python，入口SHA256为`05d3670afff57c92fc203576ba68c7e13d5527808306002300a8a1ea5a77b56f`，显式使用`--right-startup outward-hold`。原生启动即可能输出控制，不能描述为未启动电机程序。

初始双机waiting_r2，后续用户完成R2并多次重新校准。最终停止前的readiness.log确认双机曾进入policy，policy_ready.json成立，Planner PID3721369真实启动。此次物理站稳及击球未验收；用户明确反馈66初始动作不对。

重启后只读采样看到66 y约+1.36m、198约+0.16m，与Planner期望66位于较小y一侧不符；已提醒核对摆位或动捕绑定。不能仅据这些读数确定绑定错误，也没有更改坐标、配置或标记点映射。HOLD位置目标在reset时取实测位置，“人为搬动后追踪旧目标”仍只是待核实的时序假设，不能据此宣称程序无问题。

启动证据：[首次启动](device_context/yichao_original_start_20260910_002358/)、[重启](device_context/yichao_restart_20260910_003217/)。

## 保留的日志

工作站根目录：`/home/odl/codebase/yichao_v3_v9_e14fd5b`。

- 本轮会话：`output/sessions/active_20260910_003226_53c154/`，包括Predictor/姿态/就绪日志、policy_ready.json、relay.log、relay.jsonl至relay.jsonl.0003、active_result.json。
- 启动控制台：`output/restart_original_20260910_003213/`。
- 两台机载：`/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z/output/active_20260910_003226_53c154/`。

第二种方案指保留完整V9动态推理的原生HOME_HOLD候选，需用`--right-startup home-hold`显式选择。此次只停止当前程序，没有切换或启动候选，更没有恢复停student的固定关节等待实现。
