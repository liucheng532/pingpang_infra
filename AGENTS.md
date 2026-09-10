# PingPong 项目协作说明

当前代码已于2026-09-10重写：请以 yichao_v3_v9/README.md 的两个启动入口为准。以下旧会话/路径记录仅供历史参考，不可作为当前设备状态或自动启动依据。新版现场已确认站稳、移动换位和尝试接球，原 Fixed/Predictor/Controller 不修改。源码导出不含设备凭据、模型、环境、日志或旧归档。

## 项目目标与当前范围

2026-09-10 GitHub文件筛选：用户要求仅选本地原生文件，先给清单审核，不上传；已是独立GitHub仓库的目录直接ignore。已确认并排除Pingpong-Doubles-Overleaf、pingpang_planner、pingpang_deploy、pingpang_controller及local_sim2sim内两个worktree，另排除三台设备副本、vendor/assets、标定副本、环境/原始日志/证据。yichao_v3_v9只有空本地.git、无提交和remote，可选本地适配文件但不携带.git。审核白名单为152文件、2,320,849字节，含Yichao110、本地仿真7、维护工具9、本地说明26；详见[审核清单](doc/github_upload_review_20260910/README.md)及同目录include_files.txt、manifest.json。AGENTS.md和旧交接手册含凭据，原文件不上传；配置/脚本仍有内网路径和IP，排除外部模型后不是可直接完整运行的交付。仅创建本地审核材料，没有创建仓库、提交、上传或修改现有独立Git仓库；等待用户审核后再执行下一步。

2026-09-10 离场全停：用户明确动捕坏了、下班明天再测。已对本轮`active_20260910_013920_6a620a`工作站owner3764652、66 Python3591/原生3483、198 Python4051/原生3939核对PID/starttime/argv后发SIGINT，所属子进程均退出；Predictor SDK子进程由既有启动器超时退出逻辑最终SIGKILL（run.json exit=-9），不是所有组件都自然退出。再次三机复核无本任务进程、command发布者为空；保存控制台后移除`yichao_active_inputs`和`yichao_v9_active`，共享rosmaster保留。标定修复代码、当前配置副本和完整现场日志保留，本轮未实机验收站稳/回球，不自动恢复测试。软件退出不代表物理断电已确认；明天先恢复动捕再测。此条取代此前双机waiting_inputs仍运行的状态。证据`doc/device_context/yichao_end_of_day_stop_20260910_014920/`，详见[修复与离场记录](doc/YICHAO_SHARED_CALIBRATION_FIX_2026-09-10.md)。

2026-09-10 本轮启动后用户询问实时状态：`active_20260910_013920_6a620a` 的两台曾进入policy，但最新均转为 `waiting_inputs`。3秒只读采样上游/私有双侧torso均0条，私有state仍66 130条/198 131条；关节和IMU源龄约4–6ms，torso源龄约116s。Predictor stderr新增VRPN TCP连接断开，Predictor/姿态relay/Planner仍存活；Planner最新IDLE、reason=missing_private_robot_state、fault=null、commands=[]，未完成HIT。此条取代上条启动后“双机policy且输入新鲜”为当前状态；最新标定加载成功结论不变，不要求重复R2，不把waiting_inputs说成已退出/断力矩。未重启或调整输入程序。证据`doc/device_context/shared_calibration_fix_20260910_013548/input_interruption_check.json`和`user_status_latest.json`，详见[修复与当前状态](doc/YICHAO_SHARED_CALIBRATION_FIX_2026-09-10.md)。

2026-09-10 共享标定修复并按用户要求启动：Yichao Predictor 现每次启动读取部署清单的共享 `calibration_source`，复用原校验器并将原字节存为本轮副本，记录版本/刚体映射/桌面与外参/SHA256；共享缺失或非法不回退旧快照。当前9月9日22:52标定198→rigid0、66→rigid1，哈希473780a838b70be5a2198a2be720e74ec24cc6c8a552bed62b4e971965ded8f6。工作站4文件备份同步、本地副本SHA一致，本地/工作站各42测试及真实params无总线加载通过。用户随后明确改好启动，双机已重启、调试模式且安全；三设备进程/入口核对与原启动器预检后，实际启动`active_20260910_013920_6a620a`，默认66镜像OUTWARD_HOLD、198原HOME_HOLD，V9/reference未改。66原生3483/Python3591、198原生3939/Python4051已复核；tmux `yichao_active_inputs`/`yichao_v9_active`。初始waiting_r2之后，最新私有state 3秒各148条均policy且sensors_recent=true，Planner active已启动并实际写relay.jsonl。已完成R2，不再要求重复按键或重复启动；新标定已实际加载，但本轮物理站稳/接球尚未现场确认。首次诊断误订原state话题零条已由私有state复核纠正。此条取代换电时“无需机器人SSH/不自动重启”和上一全停快照；后续按本轮运行状态处理。详见[共享标定修复与启动](doc/YICHAO_SHARED_CALIBRATION_FIX_2026-09-10.md)，证据`doc/device_context/shared_calibration_fix_20260910_013548/`。

2026-09-10 Fixed正常/Yichao异常进一步核对：用户要求查两套部署差异，已取回66 Fixed最新`session_20260909_234121_616620`共27095帧、1189帧HIT，前300帧HOME_HOLD；其启动pelvis约(+.197,−.345,.729)m，Yichao今日默认约(−.441,+1.359,.753)m。最明确接入问题是Yichao冻结9月8日标定未跟随每日标定；Fixed启动读取当前共享配置（9月9日22:52），left/198→rigid0、right/66→rigid1；Yichao实录仍1/0，源码按此分配左右torso通道且用旧桌/刚体变换。配置对应错位已确认，实体刚体绑定和物理因果未复测，不宣称唯一根因/已修复。Fixed用V11 HOME/commonhold、相对X，Yichao用V9 OUTWARD/专用hold、绝对XY；新HIT残差不在startup HOME使用，前300帧双方action均<10，clip10/100差异不能解释该段。用户随后准备双机换电，已明确告知需要的代码/模型/记录全部在本地，当前不需要机器人SSH，换电不影响离线工作、不自动重启。详见[部署差异诊断](doc/FIXED_VS_YICHAO_66_DIAGNOSIS_2026-09-10.md)，证据`doc/device_context/fixed_vs_yichao_20260910_011920/`；本轮无生产修改/上传/启停。

2026-09-10 用户要求工作站二Fixed整套程序更新到本地：已同步当前Runtime/Predictor/Monitor、共享标定所属项目/SDK、66和198配套V11及数据、V01依赖/HIT Teacher，共5826普通文件、8链接、约2.53GB，独立快照与本地设备副本及末次远端SHA256均通过；原文件/校验基线备份保留，17个已有文件更新、1268新增（包含补齐依赖），无本地修改冲突。主要新增V11 Teacher Arm7 i15000、RETURN pose sync、raw-ball诊断30300及当天标定；Fixed主体与run_doubles_stack未变。没有上传、更改现场配置或启停程序。用户明确每天重标定属于正常流程；已发现Yichao输入仍冻结9月8日标定，今天默认启动stdout证实读取该旧文件，而Fixed共享配置9月9日22:52、left/right刚体ID从1/0改为0/1且table变换更新。输入版本不一致已确认，实际66错读对端姿态和站立异常因果仍待核对，不把重标定本身当故障、不盲拷新标定启动。详见[整套同步](doc/FIXED_STACK_SYNC_2026-09-10.md)，证据`doc/device_context/fixed_stack_sync_20260910_010808/`。用户随后要求进一步解释Fixed正常而Yichao异常，继续只读对比真实记录和控制链路。

2026-09-10 用户恢复只读排查，明确重点检查66默认OUTWARD_HOLD是否选错reference，并确认昨晚至少能站稳、接球。已取回历史`recovery_20260909_005802`真实HIT会话与今天`active_20260910_003226_53c154`默认版机载记录：前300帧reference位置/速度/phase/索引/帧号及镜像后的模型reference历史完全相同；按runner两次reset重建前20帧误差0，稳态为0368帧0腿部＋nominal双臂/腰零后镜像，日志move79/frame54不是HOLD实际参考源。66模型/动作资产605项哈希通过；MOVE压缩文件与原V9字节哈希不同，但155个实际使用的q/qd/fps全部相同。33个vendored deploy文件与机内原V9逐字节一致；当前ActiveRunner整段AST与历史接球基准保存入口一致。Fixed配套是V11 resume42500/HOME_HOLD/common-hold，不能混为同版或把V11参考直接换入V9。真实输入ONNX各300帧复现当前q_des最大误差4.77e-7rad；当前默认版约2秒右髋参考−0.308、目标−1.050、实测−0.957rad，异常不等于reference本身。明显输入变化为66首帧Y由历史−0.127m到今日+1.359m，模型镜像Y变为−1.359m；坐标原点/轴向、摆位、刚体绑定及朝向应继续核对，尚未证明哪项错误或唯一根因。单步换Y显著改变输出不证明可站稳。全程无生产改动、同步、总线控制或重启；前述全停保持，不自动测试。详见[默认reference核查](doc/YICHAO_DEFAULT_REFERENCE_AUDIT_2026-09-10.md)，证据`doc/device_context/deploy_reference_audit_20260910_005214/`。

2026-09-10 HOME_HOLD候选试验后用户明确“停吧，暂停一切程序”：会话`active_20260910_004059_0bea63`已软件全停。第二种方案确已实机启动，66参数`--right-startup home-hold`且保留镜像/完整V9推理；用户提供照片并报告两次R2后手脚异常抬起、不能正常待命，不能继续描述候选为“尚未上机”或“已能站稳”。只读日志确认66持续HOME_HOLD推理、目标误差接近零，当时Planner未启动/command发布者为空，不能归因于Planner发送击球或去固定让位点；具体根因尚未定位。诊断曾精确SIGSTOP工作站owner防止自动启动Planner，随后按全停指令先排队SIGINT再SIGCONT仅用于退出，owner及全部输入/就绪等待已结束；66原生8040/Python8145、198原生5063/Python5168及记录子进程均退出，三机复核无控制残留。保存控制台后移除两套Yichao tmux，共享rosmaster和日志保留。软件退出不代表已确认物理断力矩；测试及后续排查暂停，不自动重启或恢复旧方案。见[HOME_HOLD试验与全停](doc/YICHAO_HOME_CANDIDATE_TRIAL_2026-09-10.md)，停止证据`doc/device_context/yichao_all_stop_20260910_004841/`。

2026-09-10 00:37（工作站时钟）用户报告66原版两次R2后初始动作不对，先要求试第二种方案，随后明确“先kill当前程序”：已核对PID/starttime并退出会话`active_20260910_003226_53c154`的工作站Planner/输入、66原生7094/Python7199、198原生3609/Python3714及记录子进程；三机复核无本任务控制残留，command发布者为空，保存窗口输出后移除两套Yichao tmux。共享rosmaster及历史日志保留，软件退出不代表已确认物理断力矩。第二种动态HOME_HOLD候选尚未启动；不恢复固定关节等待方案。本轮原版确已加载66镜像OUTWARD_HOLD，双机后来均进入过policy，Planner真实启动并记录relay.jsonl及3片续文件；不能沿用启动中“Planner尚未启动”或此前21:58旧等待版在运行作为当前状态。66启动动作异常原因未定位；此前实测66 y约+1.36m、198约+0.16m，动捕左右顺序与Planner约定不符，已告知用户，未改坐标或绑定。启动/重启及停止详见[原版重启与停止](doc/YICHAO_ORIGINAL_RESTART_2026-09-10.md)，停止证据`doc/device_context/yichao_stop_before_candidate_20260910_003756/`。

2026-09-09 21:58 66动态待命纠正：用户实测固定关节等待版无法进入实际V9策略、站不稳，说明原版虽会首球前移动但能站稳；先要求回退、随后明确换方案。已承认停student只保持nominal不能替代动态平衡、等待却标policy错误，完整撤回固定关节等待/最终nominal补发并三机同步、41测试通过。随后在原版基线上增加显式`--right-startup home-hold`候选，仅改变66原生启动参考，保留完整动态V9循环；默认仍原66镜像OUTWARD_HOLD，198原HOME_HOLD。原install_active_runner整段AST未改；65测试、真实ONNX＋原生观察/历史/镜像600帧规定输入回放、6拍交替文件回放通过，工作站7文件/机载各3文件备份同步、SHA256及镜像副本一致。上述均未验证物理平衡/无漂移；原OUTWARD_HOLD本已锁定实测位置，不能把再次锁目标说成已找出乱动原因。当前Planner PID3597381已SIGINT退出，command发布者为空；输入/双机旧Python/原生保留，66旧进程仍加载错误等待版，最新waiting_policy_r2代次4，尚未重启/切换候选。入口磁盘SHA256 05d3670afff57c92fc203576ba68c7e13d5527808306002300a8a1ea5a77b56f；不再恢复固定关节等待方案，不自动启动物理测试。详见[动态待命候选与撤回](doc/YICHAO_DYNAMIC_HOME_CANDIDATE_2026-09-09.md)、`doc/device_context/dynamic_home_candidate_20260909/`和`doc/device_context/revert_idle66_dynamic_20260909/`。

2026-09-09 21:42 用户明确要求启动恢复后的首球待命版：三设备SSH、精确文件哈希和空闲进程核对通过；独立启动器完成双机无总线预检后，已实际启动Yichao actor199＋filter＋V9 i19000，新会话`active_20260909_213838_d7b03a`（工作站时钟）。工作站输入tmux `yichao_active_inputs`、控制tmux `yichao_v9_active`；66原生PID4346/Python4451，198原生PID5292/Python5397，PID/starttime再次核对通过。最新双机waiting_r2、HOME_HOLD，66镜像；此时Planner尚在等双机就绪，未发布本轮MOVE/HIT。已提示每台第一次R2校准、完成后第二次R2，66随后保持校准目标等首球。输入/启动日志已写入工作站`output/sessions/active_20260909_213838_d7b03a/`及`output/restored_idle_start_20260909_213827/`，Planner日志在实际启动后创建。未确认本次实际default到位或站立稳定。此条取代21:36全停状态；不要重复启动。证据：`doc/device_context/restored_idle_start_20260909_213930/`。

2026-09-09 21:36 用户报告回退版按R2后66仍移动，明确“不要回退”“把66首球移动的情况的代码改回来”：已撤销20:55的回退，恢复66首球前保持校准关节目标、首条有效MOVE受理后恢复正常V9策略，以及66当前位置HOME_HOLD和最终nominal补发。入口恢复至SHA256 1503e79e83b35c71e6e6fd9594f58994f4dc53a036956383735b6a68b63b608b；本地47测试通过，工作站5文件/机载各3文件备份同步、SHA256及本地设备副本一致。此前按最新kill要求已退出工作站输入/就绪等待、双机Python/原生及子进程，command发布者为空，两套任务tmux已移除；本轮只恢复代码，没有重启。首球待命及站立仍未实机验收，不把本次恢复说成66站不稳原因已修复。此条取代下文“首球等待已撤回”为当前版本的说明。见[首球待命记录](doc/YICHAO_FIRST_BALL_IDLE_2026-09-09.md)、`doc/device_context/restore_first_ball_idle_20260909/`及`doc/device_context/yichao_stop_20260909_213037/`。

2026-09-09 21:25 换电后用户明确要求直接启动Yichao＋V9：三设备上线且无控制进程、回退版入口哈希一致；核对旧V11两个节点API连续两次拒绝连接后注销其失效注册，保留共享rosmaster。已实际启动回退版actor199＋filter＋V9 i19000，新会话`active_20260909_212503_491bbf`，工作站输入tmux `yichao_active_inputs`、双机控制`yichao_v9_active`。两台新原生/Python均启动，最新waiting_r2，66镜像OUTWARD_HOLD、198 HOME_HOLD。未确认本次66电机使能/default或站稳，不沿用换电前mode0/512为当前结果。证据：`doc/device_context/yichao_after_battery_20260909_212547/`。

2026-09-09 21:13 用户手动启动程序后要求全部kill以换电池：实测本次是双机V11 resume42500（非Yichao），用户已明确授权退出。已核对PID/starttime并SIGKILL退出66 policy5322/g1 5161、198 policy6602/g1 6486及各自recorder子进程，避免退出路径再次校准。再次核对两台无控制进程；工作站无Planner/输入进程、command发布者为空，保存控制台后清理`v11_dual_robots`。没有改V11源码，没有重启；软件退出不代表已核实物理断电。Yichao回退代码保留。证据：`doc/device_context/battery_stop_20260909_211252/`。

2026-09-09 21:06 用户明确要求当前程序全部kill：已退出工作站Yichao输入/就绪等待及双机Python、原生g1、记录子进程，均SIGINT正常结束，无需SIGKILL；退出后再次核对两台无本任务进程、工作站command发布者为空，保存窗口输出并移除`yichao_active_inputs`/`yichao_v9_active`。共享rosmaster和历史日志保留，未重启。当前为软件全停状态，不等于已验证物理断电；回退版代码保留。证据：`doc/device_context/revert_first_ball_idle_20260909/final_stop_20260909_210612/`。

2026-09-09 回退后66无default、198每次正常：用户现场确认左右差异。只读确认66收到R2且原V9 policy持续推理；4秒私有LCM采样200条关节命令与200条原生DDS写入trace匹配，实测关节几乎不动，最大目标误差约1.43rad。进一步用/tmp只读DDS订阅器对照：66全29电机反馈mode=0，198全29电机mode=1；双方lowcmd均mode=1且Kp非零，mode_machine均5。66索引14腰pitch motorstate=512，其余0；198全部0。512含义尚未核实，不可断言某具体硬件损坏、断电或调试模式未进入。本轮仅诊断，没有改生产代码或强制使能；回退版保持，66站立修复仍已取消。证据：`doc/device_context/revert_first_ball_idle_20260909/no_response66_command_path.json`、`dds_right.json`、`dds_left.json`、`dds_comparison.json`。

2026-09-09 20:56 用户要求重启回退版机器人程序：已退出原工作站输入与双机Python/原生、确认无残留，实际启动新会话`active_20260909_205515_af836b`。入口为已回退SHA256 def5eb104c6a42eafaf15ffa97b1005ab9affda82585db85faf7b1773462abef，日志确认66镜像OUTWARD_HOLD、198 HOME_HOLD，两台均waiting_r2；首球固定姿态等待已不在新进程中。工作站`yichao_active_inputs`、控制`yichao_v9_active`，尚未本轮R2校准或击球，不代表实际站稳。见`doc/device_context/revert_first_ball_idle_20260909/restart_stop.json`与`restart_started.json`。

2026-09-09 20:55 用户明确取消66开机站不稳修复并要求退回原来：已完整撤回首球前固定校准姿态等待、66 HOME_HOLD启动切换及最终校准目标补发，恢复`first_ball_idle_20260909/before`原入口（SHA256 def5eb104c6a42eafaf15ffa97b1005ab9affda82585db85faf7b1773462abef）。66恢复镜像＋bootstrap-outward-hold、第二次R2后直接执行原V9 student；右侧66首拍、连续接球和日志等此前功能保留。本地41测试通过，工作站5文件/机载各3文件已备份同步并验SHA256、本地设备副本一致。未重启或热切换当前控制；磁盘回退下次启动生效，当前进程不能当作已加载回退版。本轮Planner此前已停止，原生/Python/输入此前保留；没有继续修复站立问题。此条取代所有“首球固定姿态补丁仍为当前代码”的说明。详见`doc/device_context/revert_first_ball_idle_20260909/`。

2026-09-09 66多次R2后站不稳：用户先确认断电重启后两台实际进入default，随后报告66多次按R、站不稳。只读日志显示当前会话66累计6次校准，最新waiting_policy_r2（calibration_generation=6），未见first MOVE accepted。已对本轮Planner PID3552578发SIGINT，command发布者为空；双机Python/原生和输入保留，没有自动重启。66私有LCM只读4秒收800关节/800IMU、零新关节指令；末帧相对nominal最大关节偏差0.2516rad（踝pitch），膝约0.15rad。固定校准关节目标没有student闭环平衡，不能把无新指令/软件校准结束当作站立稳定，具体不稳原因未确定。证据：`doc/device_context/first_ball_hold_commit_20260909/unsteady66_stage.json`、`unsteady66_joints.json`。

2026-09-09 20:43 用户说明双机断电重启且已进入调试模式，明确要求重新启动程序。已确认两台SSH在线、机载旧控制为空，退出工作站旧输入/Planner并回收断电前失联SSH窗口；启动新会话`active_20260909_204224_659c95`，输入tmux `yichao_active_inputs`、控制tmux `yichao_v9_active`。两台原生/Python均新启动，最新均waiting_r2；尚未确认66实际进入default或站立有力，未将软件就绪当作到位。此条取代20:35旧会话现状。证据：`doc/device_context/first_ball_hold_commit_20260909/powercycle_restart_stop.json`、`powercycle_restart_started.json`。

2026-09-09 20:35 用户反馈66瘫软且未实际进入default，随后明确授权全部kill并重启、说明机器人安全。先前软件policy/校准发布不证明实际到位；该问题未定位修复。已退出上一会话active_20260909_202615_b6f587的Planner、输入、双机Python/原生及子进程，无残留；保留日志和共享rosmaster。已实际重启新会话`active_20260909_203441_d201e1`，工作站tmux为`yichao_active_inputs`、双机为`yichao_v9_active`；两台新原生和Python均启动，最新均waiting_r2，尚未再次校准/发球。建议先66一次R2观察实际default，不能据软件等待/就绪宣称66有力或站稳。停止与重启证据见`doc/device_context/first_ball_hold_commit_20260909/restart_stop_2030.json`、`restarted_2034.json`。

2026-09-09 20:21 用户授权退出Fixed并自行上机测试：已核对入口PID/starttime，对工作站旧run_doubles_stack PID3479732发SIGINT，其Fixed relay/Predictor/Monitor子进程均退出，command发布者为空；随后只读核查66/198均无控制进程。未修改Fixed/V11文件、未启动Yichao，提供独立Yichao双终端active命令由用户执行。此条取代19:24“Fixed仍由他人运行”的状态快照；首球待命实机验收仍未完成。证据：`doc/device_context/first_ball_hold_commit_20260909/fixed_stop_for_user_test.json`。

2026-09-09 19:24 首球待命复查：用户确认排查此前 Yichao＋V9，当前 Fixed＋V11 是他人调试，明确不要碰。三机只读核对现运行 Fixed＋V11 resume42500，无 Yichao 控制进程；旧 Yichao 198 ROS 注册连接拒绝，未清理。此前待命补丁三机哈希一致，但尚无补丁后首球待命实机验收。本地另复现66校准已接近nominal时零次发布、可能保留旧目标的边界遗漏；已补66首球前最终nominal发布及输入门禁丢弃后的重试，本地47测试通过，三设备独立Yichao目录各3文件备份同步/SHA256及本地副本一致。发布序号不等于原生接收/到位ACK，此遗漏未被证明是此前乱动/摔倒原因。未改Fixed/V11、未启停任何现场程序或发布控制命令，仍待交接后物理复测。见[首球待命复查](doc/YICHAO_FIRST_BALL_IDLE_2026-09-09.md)与`doc/device_context/first_ball_hold_commit_20260909/sync.json`。

2026-09-09 02:38 离场停止：用户在授权重启后取消测试并明确“停止吧”。此前已重启66 Python至waiting_r2，新目录`idle66_20260909_023441`，尚无新Planner事务/击球；随后已退出工作站输入、就绪等待及两套本任务tmux，command发布者为空。66 Python PID7988及原生PID3913已SIGINT退出；198最初两路SSH不通，后管理口重连成功，核对本任务Python/原生均无残留。不能沿用临时“198无法确认”作为最终状态，也不能把软件退出当作已物理断电。日志、同步后的代码和验证证据保留，本轮无首球待命实机验收，不自动恢复测试。详见 [首次来球待命及停止记录](doc/YICHAO_FIRST_BALL_IDLE_2026-09-09.md) 和`doc/device_context/first_ball_idle_20260909/stop_*.json`。

2026-09-09 首颗来球前66待命已修改并同步：用户最终明确“不是首拍不横移，是首拍球还没来的时候别动”，其他正常。66两次R2完成后保持最后校准关节目标，持续真实状态/历史/ACK，但不运行student或发布新关节目标；第一条有效MOVE实际受理后立即进入原V9流程，首拍仍允许正常横移，后续击球/交接/返回无此等待。66启动改为当前位置HOME_HOLD并保留镜像，198启动逻辑不变；没有锁定Planner首拍目标，也没有修改actor/filter或协议。等待时control_stage=policy且control_reason=waiting_first_ball_holding_calibrated_pose，表示可受理指令而非student已推理。本地/工作站各58测试、两台无总线加载检查、双方6拍交替回放通过；工作站5文件、机载各3文件备份同步及SHA256通过。本次未启动/重启控制，66 Python与Planner仍是摔倒后退出状态，原生驱动及198/输入保留；尚未实机验证，不能称已排除摔倒原因。见 [首次来球待命](doc/YICHAO_FIRST_BALL_IDLE_2026-09-09.md) 及 `doc/device_context/first_ball_idle_20260909/`。

2026-09-09 摔倒后位置保持核查：66入口仍为bootstrap-outward-hold，198为startup-home-current；首拍改66尚未切换底层启动角色。66的HOLD目标在每次reset取当时实测位置，并非启动硬编码去−0.9125m；之后人工挪动不会自动改目标，底层持续跟踪。实测66 recorder完整2165帧全为OUTWARD_HOLD，最后一段目标−0.14956m、末帧实际−0.45644m，偏差约30.7cm；不能把所有向外移动归为正常归位，具体漂移/摔倒原因仍需区分底层跟踪、输入/镜像及现场扰动。本次只读，未改启动模式或重启。见 [核查记录](doc/YICHAO_RIGHT_FIRST_RESTART_2026-09-09.md) 与 `doc/device_context/right_first_20260909/right_hold_target_analysis.json`。

2026-09-09 02:21 后续摔倒与停止：用户报告右侧66摔倒。已立即对确切本任务Planner PID2791337和66 Python PID6546发SIGINT，后续确认两者退出，command发布者为空；没有自动重启。原生66 PID3913仍在，198 Python/原生与工作站输入保留，不能称双机断力矩。用户随后说“停了，按了一下R2”；该信息不构成物理断力矩确认，V9的R2为重新校准/阶段切换。刚才Planner日志已核实：`right_first_20260909_021026/planner.jsonl`至`.0004`共5片78,210,813字节，138.56秒，summary完整，51543条raw_input、665条relay_step、日志丢弃/写错0；有效预测0、commands_sent=0、没有MOVE/HIT，退出原因为本次SIGINT/interrupted。摔倒发生在底层policy运行期间，具体原因未确定，不得据软件就绪推断站立稳定或恢复测试。证据见 `doc/device_context/right_first_20260909/right_fall_stop_verified.json`、`fall_log_verification.json` 与 [重启记录](doc/YICHAO_RIGHT_FIRST_RESTART_2026-09-09.md)。

2026-09-09 02:19 后续恢复：用户说明66动捕现在有了；只读实测右侧上游/私有姿态均恢复，66原等待进程自动继续启动，无需重启。用户完成R2后，最新12秒采样双机state为198 598条、66 457条，两台均POLICY、valid/sensors_recent=true、control_session_invalidated=false。常驻就绪等待通过，`right_first_20260909_021026/planner_console.log` 已出现 active started=true；Planner真实日志已创建并持续写入，最新IDLE/no_valid_incoming_shot、first_hitter/next_hitter=66、fault=null、日志丢弃/写错均0。可进行本轮发球测试，尚无此次重启后的HIT/物理回球结果。此条取代下文02:14右侧无姿态、Planner未启动状态。证据见 `doc/device_context/right_first_20260909/recovered_readiness.json` 及 [重启记录](doc/YICHAO_RIGHT_FIRST_RESTART_2026-09-09.md)。

2026-09-09 02:14 最新诊断与重启：用户要求固定右侧66首拍，并明确授权重启、再次说明有绳索支撑。旧 `continuous_20260909_013859` 在首条命令前因R2重新校准永久FAULT；随后实际收到250条有效球预测，但决策/命令均0，这才是先前未接球原因。已修复首拍前无事务R2等待与右侧66固定首拍，停球超过10秒重开也由66先打；三设备备份同步/哈希通过，本地及工作站各76测试、6拍顺序66→198交替回放通过。实际退出旧FAULT Planner和双机旧Python，保留原生198 PID13850/start713997、66 PID3913/start88262及工作站输入PID2728928。新目录 `right_first_20260909_021026`：198新Python PID22253等待R2；66首次重启暴露初始torso/joint 10秒超时，另补持续初始输入等待并同步三设备、本地及工作站各32测试和双机无总线检查通过，66新Python PID6546保持waiting_initial_inputs。实测上游右侧姿态4秒0条、左侧1136条，已请用户检查66标记点/动捕识别，不能称双机可接球。`yichao_active_inputs:continuous` 是常驻就绪等待，双机POLICY后才自动启动新空事务Planner及planner.jsonl；当前该新Planner日志尚未创建，旧日志保留。右侧输入恢复后每台按两次R2启动；此前双机POLICY是旧运行状态。详见 [右侧首拍与重启记录](doc/YICHAO_RIGHT_FIRST_RESTART_2026-09-09.md)。

2026-09-09 连续接球修复已部署：用户要求减少中断性限制、超时继续接球并记录 Planner log。active 默认 `seconds=0/shots=0`；交接超过5秒、推理超过20ms不退出，错过来球只跳过该拍，短暂输入/历史间断等待恢复，V9参考HOLD可依据实测稳定位置继续交接并单独记录目标未到位。日志含36D/85D、actor/filter、命令/ACK/state及原因，后台16MiB分片。源码已备份同步三设备并验SHA256；本地/工作站各106测试通过，两台无总线加载检查通过，双端6拍回放在6秒交接延迟＋6.7cm未到位注入下完成。用户随后明确66没电后已换电重启；本轮保留198原生驱动与工作站输入，重启本任务198 Python、恢复66驱动并启动双机新policy，启动时均waiting_r2，随后双机完成两次R2进入POLICY；工作站 `yichao_active_inputs:continuous` 已启动常驻空事务Planner，没有恢复旧HIT。新日志子目录 `continuous_20260909_013859`。当前修复不是尚未同步的本地补丁，Planner日志已实际写入并分片，物理连续接球仍待本轮发球实测。详见 [连续接球与日志记录](doc/YICHAO_CONTINUOUS_PLANNER_2026-09-09.md)。

2026-09-09 本轮实机与R2最新状态：真实active已经执行198首拍HIT并完成第一次交接、66第二拍HIT，随后66让位实测约−0.850m距目标−0.9125m超过6cm完成容差，Planner因handoff_timeout退出；原生V9 OUTWARD会因reference_end进入OUTWARD_HOLD，不保证精确到位。之后用户说明R2退出，只读核对实际是两台重新校准后waiting_policy_r2，机载Python和g1均仍存活，不能称已停机/断力矩。当前不接续旧Planner事务，不自动恢复运行；运行中R2改变控制代次。工作站输入及机载进程保留。本轮超时/续接修复仍在本地验证，尚未同步/运行。详见 [active记录](doc/YICHAO_ACTIVE_TRIAL_2026-09-09.md)。

2026-09-09 active 恢复：首次 active 等待 R2 时因工作站姿态日志16 MiB上限异常退出，导致双机policy源龄过期退出；198已校准并收到第二次R2，66在校准起始退出，无Planner/HIT/交接。原生双机g1始终保留。按用户“像Fixed常驻、不要频繁退出”要求，持续日志改为分片，机载暂缺输入改为waiting_inputs保留进程并丢弃旧动作；本地/工作站各32测试，三设备备份同步/哈希通过。同会话恢复目录 `recovery_20260909_005802` 已恢复输入和两机policy，实时双机waiting_r2，已通知用户重新两次R2。最新详情见 [active记录](doc/YICHAO_ACTIVE_TRIAL_2026-09-09.md)，不能沿用首次输入进程已退出作为恢复后的现状。

2026-09-09 active 最新进展：用户明确要求聚焦核心功能、现场安全且随时可测，不再重复询问绳索/急停。独立 actor 199＋filter＋V9 i19000 active 入口已实现并同步（工作站18文件、机载各7文件，备份/SHA256一致），本地158项、工作站48项测试通过，双机无总线加载检查通过，active模式合成两拍完成。会话 `active_20260909_004525_da634c` 已真实启动双机独立 g1_control 和非shadow V9 policy；两台目前均 `waiting_r2`，已告知用户每台两次R2启动。原生驱动启动即零位过渡，不能再称“未启动电机程序”；尚未验证物理击球/交接，未冒充正式验收。工作站输入 tmux `yichao_active_inputs`、控制 tmux `yichao_v9_active` 正在运行，禁止重复启动；详情见 [active试运行记录](doc/YICHAO_ACTIVE_TRIAL_2026-09-09.md)。

2026-09-09 最新现场进展：更新接口后，用户会话 `v9_20260909_001454_a68f60` 的真实来球已推进到双机 MOVE 受理→66 CLEAR 受理→198 HIT 受理（motion 27），共一个 shot；没有输出电机。未 RETURN/交接，机器人保持原位，不能要求静止 shadow 证明物理到位。后续已修正 HIT 后等待反馈误受旧球新鲜度限制的问题，保持 HIT 前源龄/事务超时和新目标安全特征校验；同步工作站、哈希核对及本地/工作站各 8 项测试通过，两拍合成回放仍完成。后续修正未现场复测，active 未开放。详见[HIT shadow 记录](doc/YICHAO_HIT_SHADOW_2026-09-09.md)。

2026-09-09 最新接口实现：用户要求集中对齐 Yichao 输入输出，已将每拍模型决策与协议 ACK 轮询分开，锁存双机目标/shot/token，保留同拍预测更新及 HIT 前源龄检查；下一 shot 不能替换当前事务。CLEAR/RETURN 重新选择目标仍检查最新安全特征，不绕过历史间断。工作站独立目录已同步并验 SHA256，本地/工作站各 25 项测试通过；真实 actor/filter＋冻结 V9 scheduler 的合成两拍回放完成（每机一次 HIT）。本次改动未现场复测、未开启 active，原 Fixed/V11 不变。见[接口对齐最新记录](doc/YICHAO_FIXED_INTERFACE_ALIGNMENT_2026-09-08.md)。

2026-09-09 后续核对：上述有效来球会话的完整 raw_input 中，66 和 198 均有 shadow accepted，先前“尚无 66 确认”仅来自 relay 摘要，已修正。`base_history_gap` 对应 66 接收间隔 55.311 ms、源时间间隔 19.464 ms、sequence 连续；已在独立工作站 client 为状态/球订阅及命令发布启用 TCP_NODELAY，保留所有时效门禁。本地 17 项、工作站 13 项测试通过，已备份同步和核对哈希；尚未复测现场延迟。用户要求 active；当前 active 入口仍未开放，尚未启动电机，正在等待当前绳索保护及实际急停操作方式的确认。详见[记录及同步证据](doc/YICHAO_VALID_BALL_SHADOW_2026-09-09.md)。

2026-09-09 跨日更新：用户再次发球，会话 `v9_20260908_235934_09016b` 收到同一 shot 的 20 条有效预测，actor 199＋filter 生成目标（66 右侧 −0.79036 m、198 左侧 +0.24554 m）；nominal 通过过滤。发送 5 次 MOVE 协议消息（仅两个唯一 token），relay 记录 198 shadow 受理；尚无 66 确认、HIT 或完成拍次，随后因 `base_history_gap` 结束。未输出电机控制，真实输入及 active 仍未验收。详见[有效来球 shadow 记录](doc/YICHAO_VALID_BALL_SHADOW_2026-09-09.md)。不能再把“始终没有有效球/真实推理为零”作为当前结论。

2026-09-08 重启后最新清理：用户明确授权“把残留的都清一清，准备开始部署 Yichao”。已退出工作站暂停的旧双打启动器并回收其 3 个僵尸子进程，保存窗口输出后移除全死的 `v11_dual_robots` tmux 会话，逐节点两次确认连接拒绝后注销 23 个失效 ROS 节点。原 rosmaster PID/starttime 保持一致，清理后 ROS 发布、订阅及服务注册均为空；66/198 均可 SSH，未发现控制进程，工作站通过既有密钥和有线地址访问双机的启动预检也通过。本轮没有启动 Predictor、policy、g1_control 或实机控制；Yichao active 尚未开放。此记录取代下条旧 PID 尚存的现场快照，不改变源码/模型验收状态。详见[清理与启动预检记录](doc/YICHAO_PRECLEAN_2026-09-08.md)。

2026-09-08 23:25 后续：用户已明确“两台可以交给 Yichao 测试”，本轮设备测试交接授权已取得，不再重复询问测试窗口。只读实测 198 无控制进程；66 仍有 V11 g1_control PID 6712、policy PID 6870（PID 仅该时刻有效），尚未停止。等待确认 66 实际支撑/控制状态后安排退出旧控制，不能并行启动 V9 policy。配套 V9 active 的 R2 为两段校准/启动，运行中再次 R2 会重新校准，不是断力矩急停；当前 Yichao shadow 无需 R2。

2026-09-08 用户最新明确选择：Yichao 使用 `20260906_v3_planner_190_199` 的 actor 199＋安全过滤器及**配套 V9 i19000**，不是将 Planner 接入现有 V11。用户要求像 Fixed 一样两个启动命令。独立工作站已新增 `scripts/run_yichao_stack.py` 和 `scripts/start_yichao_dual_robots.sh`，自动协调会话、姿态、双机 shadow 与 relay；当前只开放 shadow，active 门禁未解除，完整组合现场复测仍待交接。输入/输出差异及使用边界见[接口对齐记录](doc/YICHAO_FIXED_INTERFACE_ALIGNMENT_2026-09-08.md)。现有 Fixed＋V11 程序仍保持不动。

**2026-09-08 用户首次完整运行现有机器人双打成功：** 用户明确说明，经现场人员指导，使用以下两个入口完成了第一次完整双打运行并成功。今后用户要运行这套现有双打时，优先给出这两个工作站终端命令，不再默认拆成逐台 SSH 手动启动所有组件，也不再把旧 V9 i19000 手册当成这套现有方案的操作入口。成功结果来源为用户现场确认；完整记录见 [首次双打成功与双入口启动](doc/EXISTING_DOUBLES_FIRST_SUCCESS_2026-09-08.md)。

两个终端均 SSH 到 `odl@172.16.3.126`，依次执行：

```bash
# 工作站终端 1：Predictor + Monitor + Fixed Planner active
cd /home/odl/codebase/pingpang_doubles_v9_runtime
python3 -B scripts/run_doubles_stack.py

# 工作站终端 2：通过 SSH/tmux 启动两台机器人的 g1_control + V11 policy active
cd /home/odl/codebase/pingpang_doubles_v9_runtime/scripts
bash start_v11_dual_robots.sh start active
```

现有组合为 **Fixed Planner + V11 resume42500**，两台机载目录为 `/home/unitree/haoran/doubles-v11-r2i299-resume42500-dual-runtime`；工作站目录仍叫 `pingpang_doubles_v9_runtime`。第二条命令省略的参数按脚本为 `normal shadow 1.0`，其中 shadow 仅指 Arm7 residual，机器人 policy 是 active。这次成功属于现有方案，不改变 Yichao V3→V9 的任务版本和验收状态。此前本次操作中的“只运行 shadow／尚无完整运行成功”记录属于较早阶段；不再作为用户当前操作经验或本次最终结果。现有控制程序仍不得被其他任务接管，未来启动仍须避开重复进程并遵守现场控制权交接。

2026-09-08 最新任务切换：用户要求先在本笔记本完成 Yichao Planner 本地 sim2sim，再继续 sim2real；已授权安装所需本地环境。独立目录为 `local_sim2sim/`，Conda 为 `yichao_sim2sim`，使用 actor 199＋safe_filter_v3＋V9 i19000 的 Isaac 双机评估入口。已实际完成 24 拍/24 次交替交接，无物理碰撞或机器人终止；击球质量仅 4/24 达标，精确训练补丁尚缺，不能作为实机验收。视频、退出处理及复测见 [本地仿真记录](doc/YICHAO_LOCAL_SIM2SIM_2026-09-08.md)，入口见 [本地仿真说明](local_sim2sim/README.md)。原实机程序继续保持不动，本地仿真不发布现场控制消息。这条更新取代下文“当前不以本机运行仿真为目标”的历史范围说明。

2026-09-08 用户交接约束：现有双打程序由用户本人运行；早先通过 `python3 -B scripts/run_doubles_stack.py --shadow` 熟悉流程，随后经现场指导完成上述 active 双打成功。用户明确要求不停止、接管或修改该程序，也不并行启动向相同 command topic 发布消息的 Yichao 程序。**Yichao 实机接入现已暂停，等待用户完成现有操作并明确交接后再继续。** 不再把当前 Fixed Planner 发布者记为来源不明，也不能沿用此前“两台空闲”作为继续启动的依据。已有代码、同步和测试结果保留，见[晚间部署记录](doc/YICHAO_ONBOARD_SHADOW_2026-09-08_EVENING.md)。

2026-09-08 晚间测试窗口已获用户明确确认：“两台机器人空闲，均能够给我测试”。不再把此前朋友占用作为当前阻塞；每次运行仍检查实际进程。已授权推进本任务双机验证，保留独立目录和唯一控制器约束；真实输入/机载 shadow 未通过前不解除 active 门禁。

最新状态（2026-09-08 晚间）见[双机 V9 实际 shadow](doc/YICHAO_ONBOARD_SHADOW_2026-09-08_EVENING.md)：两台 SSH、独立环境均已补齐，冻结 V9 i19000 机载 shadow 已实际运行并退出，无电机输出。第一轮工作站收到双机约 48–49 Hz 状态，未收到有效球；当前 pelvis 位置/朝向与测试预期存在差异，真实坐标及源龄尚未验收，active 关闭。工作站原 Predictor 当时未运行，已建立并限时运行本任务独立快照；未修改原方案。下述“66 环境待补齐/尚未运行机载 shadow”等为历史状态。

2026-09-08 晚间后续：两轮 V9 shadow 已完成；时钟首次就绪从约 13 秒缩短到 0.48 秒，但 66 仍有租约失效，尚未验收。基于三设备本地 context 发现原 Predictor 重发缓存位姿并刷新处理时间，已在独立入口增加 `predictor_freshness.py` 并同步工作站，本地/工作站测试通过。实测该修正前发现新的 Predictor 和 `/doubles_v9_real_fixed_relay` 双机 command 发布者，独立启动器已拒绝启动；当前程序运行者待用户确认，不停止现有进程。详情见上述晚间实测记录。

开发人型机器人双打乒乓球系统。本地项目：`/home/lyz/Desktop/code/PingPong`，下文本地相对路径均以此为根。

- 当前目标：把 `pingpang_planner` 的 `yichao_plan` policy 适配到现有双打 Runtime 的接口，推进 **V9 i19000 整套控制链路**下的 sim2real。用户明确暂不适配 V10/V11。
- 本机负责收集代码、模型、数据和文档，梳理接口及准备适配；当前不以本机运行仿真或训练为目标。
- Planner 工作站运行 Predictor 和双打 Planner；机器人机载电脑运行 deploy、reference scheduler 和底层 policy。
- 据用户说明，Yichao policy 尚未上真机。先核对 context、版本和接口，再依次完成离线验证、shadow 验收和满足现场条件后的 active 验证。
- 现有 Runtime 是接口参考和适配基线，不是可直接覆盖的目录。Yichao 实现须独立部署，保留他人现场基线和回滚路径。
- 2026-09-07 最新现场说明：机器人已落地、绳索保护，不再视为完全悬空。用户授权推进 default，但当前未启动本任务电机程序；具体急停按钮/操作人待明确。当前可手动执行的环境、命令及 R2 边界见 [手动启动说明](doc/YICHAO_MANUAL_START_CURRENT.md)；Yichao active 尚未实现，不得把原 Fixed Planner active 命令冒充该入口。
- 2026-09-07 最新接入：[V3 实时接口诊断 shadow](doc/YICHAO_V3_V9_RUNTIME_SHADOW_2026-09-07.md)。工作站独立方案支持 `--planner yichao --mode shadow`，只读生成特征/安全过滤/无效移动预览；不等于真实接口或闭环验收，active 仍关闭。该轮 monitor 583 次、双机 `/state` 均缺失、真实决策 0；这是当时快照，操作前重新核对。
- 2026-09-08：独立方案已增加 `target_feedback.py`，同步工作站并通过 7 项反馈关联测试；仍不构造 ACK，不开启 active。
- 2026-09-08 最新现场约束：用户说明两台机器人换电池后已开机，朋友将测试；当前不启动本任务机载 shadow、policy、g1_control 或控制客户端，不挤占、停止或修改朋友的进程。后续机载运行须重新确认测试窗口与进程状态。
- 2026-09-08 后续只读实测：[同期输入与运行版本](doc/YICHAO_LIVE_INPUT_CAPTURE_2026-09-08.md)。两台现有 policy 实际加载 V11 resume42500，尽管 ROS state 使用 v9 schema；不可据此接入本任务 V9 active。20 秒录到双机 state 各 1000 次回调和有效球预测，留存状态各 627 条、有效球 39 条通过 wire 检查；不再沿用“没有 state”的旧快照。严格诊断因旧 bridge 命令错误字段锁存，推理 0。只读节点已正常退出，未改原程序或发布控制命令；机载后续待朋友测试结束并交接。
- 2026-09-08 完整 relay 软件进展：[最新基准](doc/YICHAO_FULL_RELAY_PROGRESS_2026-09-08.md)。独立工作站已接通 actor 199＋filter→MOVE/CLEAR/HIT/RETURN→专用反馈；两拍合成轨迹回放完成，198/66 各一次 HIT。CLEAR 须明确受理后才发 HIT；真实输入及机载运行尚未验收，active 仍关闭。机载副本尚未同步这一版，朋友测试期间不启动本任务控制客户端。
- 2026-09-08 交接特征修正：完整 relay 在已有拍次交接时重新组装最新位置历史/关节特征，保留本拍击球上下文并更新 TTS；不再复用击球前整份 85D。CLEAR/RETURN 的 filter 评分尚无中途标定，仍为诊断；真实验收状态不变。
- 2026-09-08 执行接口进展见 [独立机载准备记录](doc/YICHAO_ONBOARD_MOVEMENT_PROGRESS_2026-09-08.md)：移动专用 bridge/反馈、时钟区间、禁止 LCM 发布的机载 shadow 入口已实现；本地与工作站各 98 项测试通过。两台已建 `/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z`；198 完成私有依赖导入和被动遥测编译，66 的环境补齐/编译仍待完成。机载 shadow 未运行，真实双机输入、HIT/RETURN 完整链路及 active 尚未验收，不能直接把两个目录当作同版可运行制品。

## 本地设备副本与 SSH 同步规则（2026-09-08）

用户要求把工作站二和两台机器人相关代码集中到本项目，并在本地完成代码修改后同步到对应设备。工作站范围已扩展至全部相关 codebase 项目、桌面启动入口、known-good 备份、staging 和下载目录中的相关资产，详见 [工作站补漏核查](doc/WORKSTATION_CONTEXT_AUDIT_2026-09-08.md)。目录和验证状态见 [四设备 context 索引](doc/DEVICE_CONTEXT_INDEX.md)，精确复制范围见 `tools/device_context_config.json`。

| 设备 | 本地设备目录 | 路径映射 |
| --- | --- | --- |
| 工作站二 `odl@172.16.3.126` | `odl@172.16.3.126/` | 本地目录下 `home/odl/...` 对应远端 `/home/odl/...` |
| 右侧机器人 `unitree@172.16.4.66` | `unitree@172.16.4.66_right/` | 本地目录下 `home/unitree/...` 对应远端 `/home/unitree/...` |
| 左侧机器人 `unitree@172.16.4.198` | `unitree@172.16.4.198_left/` | 本地目录下 `home/unitree/...` 对应远端 `/home/unitree/...` |

- 本开发机就是第四台设备；现有 `pingpang_planner/`、`pingpang_controller/`、`pingpang_deploy/`、`yichao_v3_v9/` 等继续保留。远端副本按设备分别保存，不把三台的同名目录合并，也不把某台的版本认作另外两台的版本。
- **对已获授权的本地代码修改，完成必要验证后必须主动通过 SSH 同步对应远端文件，并核对同步后 SHA256；不能只改本地便宣称部署完成。** 同步属于该代码修改任务的一部分，无需重复询问已经获得的同步授权。此规则是协作执行要求，不是后台文件监听器。
- 优先在本地编辑。本地设备目录中的远程副本按上表一一映射；项目根 `yichao_v3_v9/` 是本任务适配源码，须按工作站及两台机载各自实际文件布局同步受影响的适用文件，并更新对应本地设备副本。源文件、模型、冻结 vendor、机器人配置及依赖有不同作用，不得用整个根目录覆盖三套部署。
- 同步前查阅本次基线/上次同步清单，逐文件比较本地变化与远端当前内容。远端自基线变化时先拉到独立临时副本并核对/合并，不以本地覆盖远端解决冲突；本地未提交修改也不能被一次反向拉取覆盖。
- 上传采用明确文件清单，先预览差异、备份远端将替换的文件，再写入，最后逐文件核对哈希。记录目标设备、路径、同步时间、备份位置、验证结果及未同步项到 `doc/device_context/`。失败或设备离线时保留待同步清单，明确报告未一致，不能标记完成。
- 禁止把设备目录整体反向 `rsync --delete`，禁止默认上传 `.git`、运行日志、缓存、环境、编译产物、现场标定或其他机器专属配置；删除、配置变更及运行程序仍按本文件的现有授权边界办理。只同步本次获授权的修改，不顺带统一原有差异。
- 他人的 Predictor/Runtime/deploy 基线副本默认用于本地参考。修改这些现场方案仍须有对应修改授权，并遵守备份、测试窗口及互不影响约束；不能把“自动同步”当作接管朋友进程或覆盖其未提交工作的许可。本任务独立 Yichao 目录的正常源码同步按既有授权继续。
- 文件同步不会启动/重启任何 policy、g1_control、shadow 或 active，也不能替代版本、输入和现场运行验收。虚拟环境、缓存和明确的批量运行日志不在内容一致性范围；排除清单及环境版本会保留。
- 原样保留远端符号链接及 Git 信息；绝对链接在本机可能无法直接打开，应按 context 索引中的同设备路径映射查找实际目标。设备专属二进制仅作证据/依赖，不因已复制就认为能在本机执行。

## 设备入口


| 设备 | SSH 登录 | 认证凭据 | 用途与代码目录 |
| --- | --- | --- | --- |
| 个人工作站（ubun） | `ssh ubuntu@172.16.3.64` | `<REDACTED>` | 个人开发：`/home/ubuntu/Desktop/liucheng` |
| Planner 工作站 | `ssh odl@172.16.3.126` | `<REDACTED>` | 真机 Planner、Predictor、Monitor：`/home/odl/codebase` |
| Unitree 66 | `ssh unitree@172.16.4.66` | `<REDACTED>` | `table_right`，左手拍、mirrored |
| Unitree 198 | `ssh unitree@172.16.4.198` | `<REDACTED>` | `table_left`，右手拍、canonical |

电脑及工作站连接 `Archon office`（`172.16.3.*`），机器人连接 `Archon robot`（`172.16.4.*`）。据他人告知 office 网段可 ping 通 robot 网段，尚未实测。管理/SSH 地址与 `192.168.*` 有线运行地址不可混用；具体 ROS、Monitor 和青瞳地址见交接文档。

## 当前依据与身份说明

- **2026-09-07 用户指定的 Yichao URDF 已到位：** 原 `/home/lyz/Downloads/unitree_description.zip` 已校验复制到 [项目资产目录](pingpang_assets/yichao_unitree_description_20260907/)，后续使用这份，不再等待 URDF 或擅用工作站 calibration URDF/其他分支候选。压缩包 SHA256 为 `44c89eeccfbe8fe9b8d45ffb2ca522290ade93b56058e6817739cf820dadbff3`；已单独提取 [G1 main.urdf](pingpang_assets/yichao_unitree_description_20260907/unitree_description/urdf/g1/main.urdf)，29 个活动关节，SHA256 为 `2efe5bddebd5a3ba4685a085031892ed14abc16b343970876380ae61a08d76cc`。来源与复制证据见目录内 `provenance.json`。原下载文件保留；仅提取 G1 URDF，其余 mesh/机器人资产仍在原样压缩包中。
- **关节预处理更新：** 已从上述指定 URDF 生成 `yichao_v3_v9/config/joint_normalization.json`，按匹配 Controller 配置 factor=0.9 计算软限位，再按 V3 源码归一化；3 项参考对照测试通过。`joint_feedback.py` 保留物理 `q_lab_rad`，另输出 `normalized_joint_position_v3`，不能把归一化值送入 pelvis FK 或电机控制。22:24 同期 torso＋jointpos 已换算 66/198 各 805/798 条诊断 base；全链路真实输入、源龄及 ACK 仍未验收。
- **2026-09-07 基准后进展：** [实时接入进度](doc/YICHAO_V3_V9_PROGRESS_2026-09-07.md)。双机 jointpos 经只读 DDS/SSH 到工作站的顺序转换已实测通过（20 秒，66/198 各 970/985 条）；这是诊断传输，未验收实时控制源龄。当前磁盘标定为 rigid 0→right/66、rigid 1→left/198，用户初步说明一致，物理对应待确认；旧手册 0/10 映射不能直接沿用。22:03 rigid 1 曾无消息；用户挪动左侧机器人后，22:11 的 15 秒采样收到 rigid 1 共 4239 次回调，两路 707/702 条留存记录已通过新增 torso 转换模块（6 项测试通过）。当前不是第二路刚体完全缺失，下一步需同期 jointpos＋torso 换算 base 和验证完整输入。未启动 Yichao 真机控制。
- **2026-09-07 当前实施基准与回退入口：** [Yichao V3→V9 基准及进度](doc/YICHAO_V3_V9_BASELINE_2026-09-07.md)。用户确认旧文字/截图未同步修改，以 `yichao_plan@e14fd5b` 的模型、schema 和源码为准：默认 actor 199，启用 safe_filter_v3，85D 状态＋独立 2D 候选动作。工作站独立目录 `/home/odl/codebase/yichao_v3_v9_e14fd5b` 已有离线实现；真实输入、在线发布和明确 ACK 尚未完成，不能把启动选择参数或只读采样当作真机验收。文档含源码/Planner 模型快照、哈希、验证记录和后续接入顺序；用户已要求记录后继续推进部署，保留原程序与隔离约束。
- 当前 V9 部署依据：[V9 双机器人真机启动与调试手册](doc/README_V9_DOUBLES_REAL_STARTUP.md)，更新时间 2026-09-06。手册明确这是现场 debug 版本，不是已经通过安全验收的正式部署。
- **用户已确认：手册中的“184”就是 Planner 工作站 `odl@172.16.3.126`**，也就是此前的工作站二；不是新增主机。
- 新手册中的固定 commit、模型、参数和行为属于文档基线，操作前核对实机。此前 V10/V11 核查只作现场历史和资产位置参考，不作为本次 V9 启动依据。
- 新手册记录控制机青瞳网卡 `192.168.2.101`，旧记录为 `192.168.2.165`；此网络差异待实测，不擅自改配置。

## 论文 LaTeX 仓库

- 双打乒乓球论文的 LaTeX 仓库：`/home/lyz/Desktop/code/PingPong/Pingpong-Doubles-Overleaf`。
- 平常不要改动该仓库；仅在用户明确要求撰写或编辑 paper 时，按其指定范围编辑。日常部署、适配、测试和文档整理不涉及该仓库。

## Context 分布与查找入口

### 1. 新 Planner：本地 pingpang_planner

- 分支：`yichao_plan`；当前检查基准：`e14fd5ba3daa6327ac842d53fe5c4292b04e5716`。
- 当前交付：`handoff/20260906_v3_planner_190_199/`，包含 policy 190/199、冻结安全过滤器、V9 i19000 底层模型、schema、配置、仿真证据和接口说明。
- 优先阅读交付目录的 `README.md`、`DEPLOYMENT.md`、`REAL_ROBOT_INTERFACE.md`、`config/schema.json`。
- Actor 为 36D 输入、2D 双机横向位置目标输出；安全过滤器为 85D 状态加 2D 候选动作。击球归属、交接、目标保持和通信由外围逻辑负责，不能只部署 actor。
- 核心实现在 `doubles_planner/v0.py`，`v1.py` 为公共接口；V3 交付仍保留 v0/v1 命名。不要混用旧 87D、241D、294D 接口和模型。

### 2. 上游 Predictor：Planner 工作站

- 路径：`/home/odl/codebase/pingpang_predictor_dual_v9`；新手册固定 commit：`2641da9`。
- 单打 Planner 在双打中作为 Predictor 使用，负责球预测、击球点、拍面法向和拍速等信息。
- 启动入口：`TableTennis.py`；文档环境：`/home/odl/miniconda3/envs/tabletennis/bin/python`。
- 青瞳 table 标定由 Predictor 加载；Fixed Planner 消费转换后的 ball/torso，不重复应用 table 矩阵。标定矩阵见 V9 手册。
- 文档刚体映射：rigid 0 → `table_left/198`；rigid 10 → `table_right/66`。
- 重点 topic：`/doubles/ball_prediction`、`/doubles/table_left/torso_pose_origin`、`/doubles/table_right/torso_pose_origin`。
- 迁移时核对字段、坐标系、历史采样、时间戳和有效性；此目录存在现场未提交修改。

### 3. 适配基线：Planner 工作站双打 Runtime

- 路径：`/home/odl/codebase/pingpang_doubles_v9_runtime`；新手册固定 commit：`4be315c`。该目录仅作为现有方案的参考基线，Yichao 新方案另建隔离目录。
- 启动入口：`scripts/run_v9_real_fixed_relay.py`；文档环境：`/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python`。
- 默认 shadow，增加 `--active` 才正式下发；状态 topic 为 `/doubles/v9_fixed_relay/status`。
- Fixed Planner 使用 `hit`、`hold`、`return`；文档目标为 left home=+0.20/outward=+0.9125，right home=-0.20/outward=-0.9125。
- RETURN 持续发布，直到机器人 phase 进入 RETURN/HOME_HOLD 完成确认。这是现有执行语义，不直接等同于 Yichao policy 的目标定义，需适配核对。
- 此目录存在未提交修改；实现新方案时保留这些现场内容，不覆盖、reset、clean 或直接切换其分支。
- **2026-09-08 用户指出这套是当前稳定可运行方案的参考候选，应作为 Yichao sim2real 的优先对照。** 已按现场工作树复制到 `odl@172.16.3.126/home/odl/codebase/pingpang_doubles_v9_runtime/`；参考入口见 [现有可运行 Planner 参考索引](doc/FIXED_PLANNER_REFERENCE_2026-09-08.md)。保留未提交修改，不能只按 HEAD 重建。稳定性属于用户说明；确切成功运行组合仍需按启动配置和机载模型对应，目录名 V9 本身不足以判断底层版本。
- 据用户说明目录名 V9 源于 WBC 版本命名，不能据目录名判断现场当前加载的模型。

### 4. V9 下游执行及部署资产：两台机器人

- 两台的当前任务指定目录：`/home/unitree/haoran/doubles-v9-real-runtime`；新手册固定 deploy commit：`347890e`。
- V9 student：iteration 19000，ABI `1666 -> 29`。
- Checkpoint SHA256：`2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a`。
- ONNX SHA256：`7e7e26d71895eb900c7b95fa2cd2fb69cbe8207586869390138c6f758848cae7`。
- ONNX 相对部署目录：`policy/v9_model19000/student_v9_m14500_timedhandoff_1666_model19000.onnx`。
- 启动入口：`g1_gym_deploy/scripts/start_v9_g1_control.sh`、`start_v9_policy.sh`。
- 用户转述 controller 同学：部署数据、checkpoint、运行接口等资产在机载电脑；已在 V10/V11 目录确认部署代码、模型和动作库存在，但不代表 V9 各项已核验。
- 当前优先查 V9 目录的 `g1_gym_deploy/scripts/deploy_policy.py`、`utils/planner_ros_bridge.py`、reference scheduler、镜像逻辑、runtime motion config、`policy/` 和 `data/`；具体位置和内容需在 V9 中核实。
- 必须成套使用 V9 scheduler、sidecar 和 ONNX，禁止在 V10/V11 Runtime 中只替换为 V9 模型或保留其他代 reference 语义。
- 机载 deployment 不等同于 Isaac 训练源码；精确训练补丁若机载未保存，再向 Yichao 训练工作树追溯。
- 历史快照：2026-09-06 两台都保存 V10 和 `doubles-v11-r2i299-resume42500-dual-runtime`；198 当时进程使用 V11 resume42500，66 未查到对应 `deploy_policy.py`。这是他人现有方案，不因此自动停止或切换为 V9。

### 5. V9 角色、启动状态与通信

| 机器人 | ROS角色 | `<REDACTED>` | V9 冷启动状态 | 首拍 |
| --- | --- | `<REDACTED>` | --- | --- |
| 198 | `table_left` | `<REDACTED>` | HOME侧，HOME_HOLD | 是 |
| 66 | `table_right` | `<REDACTED>` | OUTWARD侧，OUTWARD_HOLD | 否 |

- 198：ROS_IP `192.168.124.164`，LCM `239.255.76.198:7667`；66：ROS_IP `192.168.123.164`，LCM `239.255.76.66:7667`。编号、角色、镜像和 LCM 组须成套使用，两台不能共享同一 LCM multicast。
- 机器人访问 ROS master：`http://192.168.123.165:11311`；Planner 工作站本地访问：`http://127.0.0.1:11311`。
- 文档循环从 198 HIT 开始，向外让位、66 RETURN/HOME，再由 66 HIT，交替进行。两拍间隔超过 10 秒时选择 `abs(pelvis_y)` 更小的有效机器人，不是永远固定首拍。
- 上述 topic/LCM/端口是现有 V9 基线参数，不是允许新方案与现有程序并发占用同一资源的授权。

### 6. 本地 Controller 与 Deploy 仓库

- `pingpang_controller` 当前检出 `main`，仅有 README，不表示其他分支未下载。
- 本地分支 `codex/v9-planner-integration`，HEAD `47359de`：匹配 Yichao 交付基础仿真控制器，含 `V9_CONTROLLER_CONTRACT.json`。
- 本地分支 `base-ball-contact-loop`，HEAD `a412091`：包含 G1 URDF/mesh 候选资产，与训练资产一致性尚未确认。
- 三条 20260830 较新 controller 分支已拉取，但不作为此次 V9 基线，不因日期较新替代匹配集成。
- `pingpang_deploy` 分支 `feature/v9-doubles-real-runtime`，已检查 HEAD `347890e`，与新手册 deploy commit 对应。用于阅读及准备修改，不能默认与机载文件一致。
- 开发前核对本地检出分支；未经授权不改变他人的远程工作树。

### 7. 本地动作数据与核查资料

- 原始动作：`doc/motion_lib/`。
- `pingpang_assets/0718-move-160-80hz`：155 条移动动作，用 Planner 自带脚本从 50 Hz 数据生成。
- `pingpang_assets/0302_combined`：链接到 `doc/motion_lib/0302_combined`，441 条击球动作。
- `doc/motion_lib/0227_combined`：893 条备用动作，当前 V3 配置不使用。
- 结构及转换检查已通过，尚未证明与训练端和机载 V9 数据逐字节一致。详细哈希和核查证据在 `doc/`，不要将已到位数据误记为缺失。

## 当前待核对事项

1. 目标设备 V9 commit、模型、sidecar、scheduler、动作库与运行进程；控制机青瞳网卡新旧记录差异。
2. 现场数据构造 36D/85D 输入的坐标系、左右命名、历史采样、关节顺序、归一化和时间戳。
3. Yichao 横向目标与 V9 hit/hold/return、目标保持和交接确认的衔接。
4. 精确仿真复现涉及的 Controller 未提交补丁：基础 HEAD 已有，但匹配交付哈希的 `distill_commands.py` 尚未找到。
5. Planner 工作站的 V3 推理依赖与新方案所需资源；本机 Isaac Lab 依赖升级暂不作为优先任务。

## 隔离部署与互不影响原则

- 用户明确：机器人和电脑全天候开机，他人方案可能持续运行。开机、可 SSH 或进程长期运行均不代表设备空闲、进程无用或允许接管。
- 不改动现有部署方案，不覆盖他人的代码、模型、sidecar、标定、动作库、启动脚本、日志或配置；不停止、重启、挂起或调试附加到他人的运行进程。
- Yichao 方案使用独立目录/工作树、配置、启动入口、日志、输出和依赖环境；需要参考现场代码时先只读核对，再建立独立副本，不在原目录直接修改。
- 独立目录不等于运行隔离。启动前核查进程、GPU/CPU/内存负载、端口、ROS master/节点名/topic、LCM multicast、设备连接等资源，避免节点重名、抢占端口、覆盖日志和算力挤占影响现有测试。
- 新方案离线/replay/shadow 默认不向现有控制 topic、LCM 或机器人设备发布指令；复用现场输入只读订阅且不干扰现有发布者。需要并行通信时使用独立 namespace/topic/端口，并验证不会进入他人控制链路。
- 实机控制不能靠不同目录或 topic 实现双控制器安全并发。同一机器人存在他人控制程序时，不启动新的 active 控制器；先获得明确的设备测试窗口及控制权交接授权。
- 如确需修改他人程序、共享环境、服务或配置，或停止现有进程/占用其资源，先给出具体改动、影响范围、备份与回滚方案，取得用户明确授权再执行；不能将“部署 Yichao”理解为这类操作的默认许可。

## 操作与凭据约束

- **2026-09-09 用户明确要求：以后每次启动都记录 log。** Yichao 默认保留独立会话目录，记录 Predictor/姿态输入、Planner决策、command/ACK/双机state、启动退出原因及机载执行数据。持续日志按段保存，不能因诊断日志大小限制退出控制链路；报告实际日志路径。此要求适用于后续启动，不需要重复询问是否录制。


- Agent 可直接使用本文件已记录的密码 SSH 登录所有列出的设备，无需再次确认或用户手动输入；已有公钥认证也可使用。执行前自行核对目标主机、目录和机器人角色。
- 用户授权在本文件保留上述密码；不扩展至其他文件、源码、脚本、日志或 Git 历史。不要在命令行参数或可记录输出中回显凭据。
- SSH 登录授权不等于所有远程操作授权。修改 SSH、公钥认证、防火墙、网络或账户配置，以及安装软件、修改系统文件、重启服务或设备、删除数据，须先说明影响并取得用户授权。
- 上述两个 Planner 工作站项目存在现场未提交修改；修改前先备份并确认差异，不执行 `git reset`、`git clean` 或直接切换分支。不要将本地开发实现与现场部署视为同一版本。
- 一键启动在 Planner 工作站执行。启动前确认机器人已扶正或吊起、急停和 R2 可用，并检查旧进程；只有 shadow 验收完成、现场安全条件满足后才能执行 active 运行。具体步骤见交接文档。
- 核查报告、证据和目录快照存放在 `doc/`；本文件仅保留核心信息、约束和文档入口。区分实测结果、用户说明及尚未验证的信息。

## V9 实机操作补充约束

- V9 手册记录 66/right 在 RETURN 阶段发生闭环发散和摔倒；deploy 尚无 q_des rate limit、姿态/高度 watchdog 和 tracking-error 自动 disarm，核查前不能假定已有这些保护。
- 必须先检查保护条件、版本、模型与重复进程。Shadow 不校准、不发布 q_des，同一机器人不能同时运行 shadow 和 active policy。
- 没有吊装或保护时不得直接运行完整 active 循环。R2、验收、启动/停止顺序以 V9 手册为准，并受上述隔离与授权规则约束。
- 停止程序前确认确切 PID；禁止模糊全局 pkill 误伤其他程序。只处理本任务获授权的进程。

## 文档入口

- [当前 V9 真机基线、网络、启动、验收及已知限制](doc/README_V9_DOUBLES_REAL_STARTUP.md)
- [V3 Planner 交付说明](pingpang_planner/handoff/20260906_v3_planner_190_199/README.md)
- [本地仿真条件、Controller 分支、动作资产定位](doc/LOCAL_ISAACLAB_READINESS_2026-09-06.md)
- [机载 Controller 核查：V10/V11 历史现场证据](doc/ROBOT_CONTROLLER_INVENTORY_2026-09-06.md)
- [2026-09-05 工作站二核查](doc/WORKSTATION2_VERIFICATION_2026-09-05.md)
- [旧版双打系统新人交接](doc/README_DOUBLES_ONBOARDING.md)

当前 context 按“Predictor 数据 → Yichao policy 与安全/交接逻辑 → Runtime 通信与调度 → 机载 V9 执行”组织。区分用户说明、文档基线、实测文件、运行进程及验证结果；详细证据放在 doc，发现冲突先核对。
