# 66动态待命候选与错误方案撤回

用户现场反馈：恢复的固定关节等待版无法让66进入实际V9策略并站稳；原版虽在首球前移动，但能够进入V9 policy并自己站稳。用户先要求回退，随后要求换一种方案。当前默认启动已恢复原版行为，另提供显式选择的动态HOME_HOLD候选，未自动重启或上机验证候选。

## 已撤回的错误实现

首球前调用`_wait_first_prepare()`而不返回原控制循环，会停止student推理和新关节目标发布。关节PD保持校准目标不能替代V9根据身体状态持续调整的动态控制；把该等待阶段标成`policy`也会误导就绪判断。此前47项测试验证了暂停行为，却没有证明物理站立能力，不能作为该改法可用于实机待命的依据。该实现和最终nominal补发已完整撤回。

本轮日志证实66收到两次R2后进入`waiting_first_ball_holding_calibrated_pose`，多次R2又重新校准，最后为`waiting_policy_r2`、校准代次4。没有首条MOVE受理日志。198仍持续策略推理并记录动作。证据：[现场阶段与停止Planner](device_context/idle66_dynamic_diagnosis_20260909_214407/stop_and_status.json)。

回退先恢复到改动前入口`def5eb104c6a42eafaf15ffa97b1005ab9affda82585db85faf7b1773462abef`，本地41测试、三设备备份同步及哈希核对完成。见[完整回退](device_context/revert_idle66_dynamic_20260909/validation.json)。之后只在该基线上增加下述可选启动参考；原`install_active_runner()`整段AST与回退基线一致，原校准、50Hz循环和发布路径保持不变。

## 候选方案与局限

原版OUTWARD_HOLD本来就把全局位置目标取为reset时的实测位置。因此，再次锁定相同位置不能证明解决了漂移。候选只改变66启动给V9策略的原生参考：从OUTWARD_HOLD改为原生HOME_HOLD，使用实测当前位置作为home，保留66完整镜像。198已经使用HOME_HOLD启动，但这不是66候选通过物理验证的证据。

HOME_HOLD与OUTWARD_HOLD的参考关节姿态不同：原生HOME_HOLD使用返回动作末帧的下肢参考和击球准备上肢参考；OUTWARD_HOLD使用准备姿态下肢与nominal上肢。这些是送给策略的参考，电机关节目标仍由每轮V9推理计算。未修改模型、冻结scheduler、镜像数学、actor/filter或协议。

两次R2完成后立即进入原V9策略循环，无球也持续观察、推理、控制和记录。收到有效MOVE后，原receiver接管目标并可正常横移、HIT、交接与RETURN；没有新等待循环。再次R2按所选启动参考重新校准。`policy`阶段仅在实际推理成功后设置，不证明物理到位或站稳。

候选尚未完成物理仿真闭环或实机站立验证。仍可能漂移；现有测试不能证明新参考解决了原来的移动原因。用户之前要求等待时保持原位，这一物理目标目前仍未验收。

## 使用

工作站独立目录`/home/odl/codebase/yichao_v3_v9_e14fd5b`，旧控制退出后才能启动新会话：

```bash
# 终端1：原命令
python3 -B scripts/run_yichao_stack.py start active

# 终端2：显式选择66动态HOME_HOLD候选
bash scripts/start_yichao_dual_robots.sh start active --right-startup home-hold
```

省略`--right-startup`仍使用原版66镜像OUTWARD_HOLD。该选项不会恢复固定关节等待；两种选择均运行完整V9策略。启动选择写入机载控制台和工作站`robots_requested.json`、`robots_result.json`，不会热切换旧进程。

## 验证及同步

- 65项本地测试通过。直接执行原控制循环的600帧测试确认无球时每帧调用策略与控制，注入位置变化后全局目标仍固定；过期MOVE拒绝，首条有效MOVE可从−0.2m移动到−0.65m并进入HIT。该测试使用规定的传感器输入与替代策略，不是动力学仿真。
- 真实V9 ONNX、原生172D观察生成、1666D历史和66镜像链路完成600帧输入回放，输出全部有限，随规定输入变化；锁定目标误差约7.45e−9m。ROS/LCM入口禁止调用。该回放没有用物理引擎根据输出计算运动，不能证明平衡能力。
- actor199＋filter＋原生scheduler的6拍文件回放完成66→198交替；注入6秒交接延迟、6.7cm未到位仍完成。此回放不运行物理机器人动力学。
- 工作站7文件、每台机载3文件逐文件核对基线、备份同步、SHA256及本地设备副本一致；没有待同步文件。备份前缀`backups/dynamic_home_candidate_20260909T135624Z`。

证据：[验证](device_context/dynamic_home_candidate_20260909/validation.json)、[同步](device_context/dynamic_home_candidate_20260909/sync.json)、[差异](device_context/dynamic_home_candidate_20260909/candidate.diff)、[真实模型输入回放](device_context/dynamic_home_candidate_20260909/student_probe.json)。

## 当前进程

已对本会话`active_20260909_213838_d7b03a`的Planner PID3597381发SIGINT并核对退出，command发布者为空。诊断时曾向工作站owner发送SIGSTOP以避免就绪转换期间新启动Planner；之后读取状态为S，最新核对未挂起，也没有重新启动Planner。输入保留，双机Python/原生仍在，66旧进程仍加载固定关节等待版，最后等待第二次R2；磁盘回退和候选不会热加载。没有自动重启机器人、没有启用候选，不能把当前现场进程描述成已经运行原版或新候选。见[最终运行核对](device_context/dynamic_home_candidate_20260909/runtime_not_restarted.json)。
