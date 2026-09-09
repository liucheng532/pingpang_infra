# 66默认OUTWARD_HOLD：deploy与reference核查

后续同步Fixed时已确认输入版本错位：用户说明每天重新标定；Fixed当前共享配置更新于9月9日22:52，而今天Yichao默认启动日志确实加载9月8日冻结标定，左右刚体ID与当天配置相反，球桌变换也不同。这个新证据补充了下文“坐标变化待查”的结论；重标定本身是正常流程。详见[同步与标定证据](FIXED_STACK_SYNC_2026-09-10.md)。本轮仍未改生产配置、重启或完成物理根因验证。

用户报告66两次R2后手脚张开，要求重点检查默认原版是否选错reference，并以“昨晚至少能站稳、能够接球”为基准。本次恢复只读排查；没有修改或同步生产代码，没有启动机器人、发送控制消息或恢复测试。此前固定关节等待方案仍已撤回。

**结论：没有发现今天默认版换错待命reference。** 找回9月9日凌晨有真实HIT记录的Yichao会话，与9月10日默认原版的前300帧逐项对照，reference关节位置、速度、阶段、move索引、帧号、输入模型的镜像参考历史全部相同，最大差值为0。异常关节目标可以由今天保存的输入和原V9 ONNX离线复现；参考一致不等于站立问题已经解决，也不能据此断言整个程序没有问题。

## 对照基准与证据

证据目录：[deploy_reference_audit_20260910_005214](device_context/deploy_reference_audit_20260910_005214/)。

| 对照项 | 记录 |
|---|---|
| 有击球记录的基准 | `active_20260909_004525_da634c/recovery_20260909_005802/policy_66/session_20260909_005816_860582` |
| 基准完整有效帧 | 25,461帧，sequence 0–25460；包含66 HIT/让位/返回 |
| 辅助基准 | `continuous_20260909_013859/policy_66/session_20260909_013935_476719`，30,000帧OUTWARD_HOLD；该记录内没有HIT |
| 今天默认版 | `active_20260910_003226_53c154/policy_66/session_20260910_003251_925683` |
| 今天默认版完整有效帧 | 1,244帧，全部phase_id=3 OUTWARD_HOLD，没有HIT |

“昨晚能站稳、接球”来源于用户现场确认；历史软件HIT记录本身不证明触球质量。这是找回的可核对运行基准，不能独立确定用户所指正常表现的每个具体时间点。照片来自随后HOME_HOLD候选试验，不能把其所有实测数值套在默认版上；本报告核心使用默认版自己的记录。

采集脚本只读现存metadata与frames.npy，保留完整帧计数/状态摘要，下载开头300帧、间隔采样、状态切换附近和末尾200帧。前6秒逐帧比较使用完整sequence 0–299。详见[参考核对与回放结果](device_context/deploy_reference_audit_20260910_005214/reference_audit.json)。

## 默认reference实际是什么

第一次R2校准到nominal/default关节姿态；第二次R2重新reset观察历史与scheduler，随后持续运行V9 student。动态待命参考并不等同于校准用nominal，因此不能要求每个关节永久等于default值；但这也不能解释或合理化用户报告的异常大幅张开姿态。

66默认参数为`--mirror-left-hand --bootstrap-outward-hold`。待命参考的来源是：

- `assets/0302_combined/0302_combined-0368:v0/motion.npz`，零基帧号0；HIT库索引368，50 Hz。
- 稳态OUTWARD_HOLD取该帧的腿部关节，腰三个关节置0，双臂使用nominal值，再按关节名称做66左右镜像。
- 没有按整段0368击球动画播放，也没有直接把MOVE文件末帧当作稳态OUTWARD_HOLD。
- 启动前约0.3秒存在参考平滑过渡；phase向量约0.1秒完成切换。两次历史基准和今天的过渡序列相同。
- 本次记录里`move_motion_index=79/reference_step=54`是保留的移动段索引/帧号。**在OUTWARD_HOLD中，它不代表正在跟踪0079的第54帧姿态。** 应以`reference_joint_pos`和真正输入模型的command为准。

按照原runner“R2前reset一次、第二次R2后再reset一次”的顺序离线重建，三个会话启动前20帧reference与实录最大差值均为0。当前运行的启动过渡起点可对应0004返回动作末帧加工后的HOME参考，最终过渡到上述0368腿部＋nominal双臂。只模拟一次reset会得到不同过渡起点，不能据这种不完整复现判断部署选错动作。

参考文件SHA256为`8ab389ad9617f0c0efe42ff7fd21ffbdb3a90f9ad7134123baec90814bc0afdb`，与锁定资产一致。66当前模型、hit/move库及相关文件共605项通过机内manifest哈希核对。155个MOVE npz与原V9目录的压缩文件字节哈希不同，但逐一核查了deploy实际读取的`joint_pos/joint_vel/fps`，**155个全部数值相同**；不能将压缩文件哈希不同称为运动参考换了。证据：[文件检查](device_context/deploy_reference_audit_20260910_005214/all_motion_hashes.json)、[MOVE数值核对](device_context/deploy_reference_audit_20260910_005214/move_bank_numeric_comparison.json)。

## deploy与Fixed配套版本的差别

66机内Yichao的33个vendored deploy文件与`doubles-v9-real-runtime`逐字节相同，包含LCMAgent、scheduler、history、mirror、joint mapping和runner。私有原生`g1_control`二进制与该V9基线哈希相同；V9与Fixed/V11的`g1_control.cpp`源码哈希也相同。没有完成原生二进制到源码的可重复构建证明，不把源码相同扩展为两套全部二进制相同。

独立Yichao入口不是完全无改动：增加了私有总线/会话、命令受理与输入等待等。与有击球记录后、连续运行改动前保存的66入口相比，当前`install_active_runner`整段AST相同（含两段R2、校准、推理封装）；额外差异主要是初始输入等待、可选HOME_HOLD参数与启动日志，默认启动参数仍相同。详见[入口比较](device_context/deploy_reference_audit_20260910_005214/baseline_entry_comparison.json)及同目录`baseline_entry.diff`。

| 项目 | Yichao默认66 | Fixed配套66 |
|---|---|---|
| 底层student | V9 i19000 | V11 resume42500 |
| 默认启动 | 镜像OUTWARD_HOLD | 镜像HOME_HOLD、startup-relative-x |
| HOLD参考 | V9 0368腿部＋nominal双臂；HOME另有规则 | V11专用common hold：source29末帧腿部＋0368上肢、腰归零 |
| 击球路径 | V9 student及配套scheduler | V11移动student＋HIT teacher；常规入口Arm7 residual为shadow |
| 关节映射、增益和action scale配置 | 与Fixed对应内容相同 | 与V9对应内容相同 |

因此不能说两套deploy完全一样，也不能将Fixed的common hold单独换进V9便认为正确。V11 hold文件是`source29_0368_upright_fk_v1`，受其模型合同约束；本轮没有替换它。

## 异常输出与下一处应查的输入变化

默认版sequence 100（按50 Hz约2秒）右髋侧摆：reference为−0.308 rad（约−17.6°），模型输出q_des为−1.050 rad（约−60.1°），实际反馈约−0.957 rad（约−54.8°）。历史基准相同sequence输出为−0.334 rad（约−19.1°）。这直接体现了参考角度与异常电机目标的差别，不是把照片姿态臆测成某个reference。

使用同一V9 ONNX在本机逐帧回放历史和当前各300帧真实输入，当前q_des复现最大误差为`4.77e-7 rad`，历史为`2.38e-7 rad`。这表明无需Planner命令或更换模型即可从保存的输入复现这些目标；它不能证明输入物理含义正确，亦不是闭环稳定性验证。

两次启动首帧的明显输入差别：

| 项目 | 历史击球基准 | 今天默认版 |
|---|---:|---:|
| 66实测pelvis X | +0.269 m | −0.441 m |
| 66实测pelvis Y | −0.127 m | +1.359 m |
| 输入模型的镜像Y | +0.127 m | −1.359 m |
| torso Z | +0.739 m | +0.794 m |

V9观察包含绝对task-anchor坐标；R2锁定当前位置目标不会把此坐标自动归零。镜像后的Y从正侧变成大幅负侧，是应优先追查的输入变化。模型sidecar的训练home_y_range为[0,0.4]、outward_target_y为+0.9125，但没有完整运行分布，不能将这些标称值当作硬性有效范围或直接认定唯一根因。

只做单步输入敏感性分析：将今天首帧参考历史替换成昨晚参考历史，输出不变（本来就相同）；仅将task-anchor Y历史替换为昨晚数值，最大关节目标变化约0.747 rad。该分析保留了原来的其他传感器与动作历史，不是物理回放，也没有证明改Y就能站稳。**下一步应核对实际摆位、动捕坐标原点/轴向、66刚体绑定及朝向的一致性；当前不能断言其中任何一项已错误，更不能盲目修改符号或零点。**

本轮仅保存诊断脚本、证据与文档。默认reference的回归嫌疑已通过历史逐帧对照明显降低；实机姿态异常根因与站立恢复尚未完成。
