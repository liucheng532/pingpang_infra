# 66动态HOME_HOLD候选实机试验与全停

用户明确要求启动第二种方案，随后报告66两次R2后手舞足蹈、手脚抬起，并提供现场照片要求检查。检查期间用户最终要求“停吧，暂停一切程序”。本次已停止全部任务程序与后续排查，候选未通过稳定待命验收，根因尚未定位。

## 已执行的启动

三机在线、无本任务控制残留，关键入口哈希与本地一致；独立启动器完成双机无总线检查后，启动新会话`active_20260910_004059_0bea63`。

66实际启动参数包含`--right-startup home-hold`，底层为`--mirror-left-hand --startup-home-current`，保留V9 i19000完整推理。198仍为原生HOME_HOLD。没有恢复固定关节等待，也没有修改模型或冻结scheduler。

启动证据：[启动记录](device_context/yichao_home_candidate_start_20260910_004227/launched.json)。工作站启动控制台：`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/home_candidate_20260910_004048/`。

## 现场反馈与检查范围

用户照片显示66双腿明显外张、持拍手抬起，并明确报告待命动作异常。不能把照片解释为已能自主稳定站立，也不能从一张照片确认绳索受力比例或电机故障。

只读控制台确认66处于HOME_HOLD，V9推理和记录持续进行，当前位置目标误差接近零。检查时Planner没有启动，command发布者为空。为防双机就绪后自动进入接球调度，精确核对PID/starttime后暂时SIGSTOP工作站启动器PID3726100，输入子进程和机载控制保留。

已开始获取参考关节、策略输出、实测关节及输入的只读记录；用户随即要求暂停。尚未完成根因定位，未修改传感器绑定、坐标、关节映射或控制策略。不能认定异常来自某个硬件部件、镜像错误或某一个输入维度。

诊断证据：[暂停自动调度及现场控制台](device_context/home66_abnormal_20260910_004602/paused_and_console.json)、[诊断目录](device_context/home66_abnormal_20260910_004602/)。

## 最终软件全停

先重新枚举三机进程，核对完整argv、PID/starttime与本任务目录，再执行退出：

- 工作站owner3726100：排队SIGINT后SIGCONT，使此前暂停的owner执行退出；输入、Predictor及就绪等待均结束。
- 66：原生8040、Python8145及记录子进程结束。
- 198：原生5063、Python5168及记录子进程结束。
- 退出后再次枚举三机进程，无本任务控制残留，未追加SIGTERM或SIGKILL。
- command发布者为空，保存窗口输出后移除`yichao_active_inputs`和`yichao_v9_active`。共享rosmaster PID1150539/start12294544保持，日志保留。

证据：[进程退出](device_context/yichao_all_stop_20260910_004841/stopped.json)、[最终核对](device_context/yichao_all_stop_20260910_004841/final_verified.json)。软件退出不代表已确认物理断力矩。用户要求暂停一切程序，不自动恢复控制、试验或后续排查。

日志根目录：工作站`/home/odl/codebase/yichao_v3_v9_e14fd5b/output/sessions/active_20260910_004059_0bea63/`，机载`/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z/output/active_20260910_004059_0bea63/`。本轮未创建Planner relay日志，输入与机载记录保留；不宣称日志完整性或物理行为已验收。
