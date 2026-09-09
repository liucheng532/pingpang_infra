# Fixed正常、Yichao启动姿态异常：66对照结论

后续：用户已授权修复和启动，现已改为每次启动读取共享标定，并启动默认原版新会话，双机已进入policy。详见[修复与本轮启动](YICHAO_SHARED_CALIBRATION_FIX_2026-09-10.md)。以下是修改前诊断，不能继续当作当前全停或未修复状态。

目前最明确的部署问题在输入标定：**Yichao读取冻结的9月8日标定，未跟随现场每天的重新标定。** Fixed标准启动入口读取当前共享配置，两者的左右刚体ID与球桌坐标变换已经不同。这个配置/加载差别有源码和Yichao实际启动日志证据；它是需要优先纠正的问题，但没有经过输入修正后的物理复测，不能称为已证实的唯一摔倒/异常姿态根因。

用户说明每天重新标定是正常流程，本报告不把重新标定本身视为异常。用户随后准备给双机换电：必要代码、模型和记录均已在本地，此阶段不再需要机器人SSH，断电换电不影响离线排查；本轮不自动重启。

## 输入链路中的确定差别

Fixed的`scripts/run_doubles_stack.py`将`PINGPANG_CALIBRATION_CONFIG`指向共享文件：

`/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/calib/calibration_config.json`

当前文件更新于`2026-09-09T22:52:07+08:00`，left/198使用刚体0，right/66使用刚体1。

Yichao的`tools/run_predictor_snapshot.py`则将同一环境变量指向：

`/home/odl/codebase/yichao_v3_v9_e14fd5b/predictor_snapshot_20260908/calibration_config.json`

今天默认会话`active_20260910_003226_53c154`的Predictor stdout明确记录加载此文件、版本为`2026-09-08T16:45:26+08:00`，left/198为刚体1、right/66为刚体0。`params.py`用配置填充`DOUBLES_TABLE_LEFT/RIGHT_TRACKER_RIGID_ID`及tracker外参；`Mocap.py`依据这些ID把位姿发布到左右robot topic。因此这不只是文件日期不同：**按当天配置分配关系，Yichao左右姿态通道对应反了，同时沿用了旧桌面/刚体变换。** 尚未通过现场单独移动刚体测试证明ID与实体的绑定，不能把配置推导写成已完成物理识别验证。

证据：[当前/旧标定差别](device_context/fixed_stack_sync_20260910_010808/calibration_version_mismatch.json)、[实际加载路径与启动日志](device_context/fixed_stack_sync_20260910_010808/yichao_calibration_provenance.json)。

第一次R2的最终关节校准目标是nominal，主要根据关节位置缓慢调整。第二次R2进入student之后，输入还包括动捕推导的pelvis位置和朝向；输入来源/坐标错误会直接影响策略生成的关节目标。这个机制与“第一次default正常，第二次后异常张开”的时间点相符，不需要Planner先发MOVE/HIT才会出现。

## 真实记录对照

取回66的Fixed/V11记录`session_20260909_234121_616620`：27,095帧，包含1,189帧HIT以及让位、返回和待命。前300帧都是HOME_HOLD。是否真实接触球及站立稳定仍以用户现场反馈为准，阶段记录本身不是物理验收。

与今天默认Yichao记录对比：

| 项目 | Fixed，9月9日23:41会话 | Yichao，9月10日00:32会话 |
|---|---:|---:|
| 启动pelvis X | +0.197 m | −0.441 m |
| 启动pelvis Y | −0.345 m | +1.359 m |
| 前300帧状态 | HOME_HOLD | OUTWARD_HOLD |
| 第100帧右髋侧摆reference | −0.210 rad | −0.308 rad |
| 第100帧右髋侧摆q_des | −0.090 rad | −1.050 rad |
| 第100帧右髋侧摆实测 | −0.174 rad | −0.957 rad |

两次现场运行不是同条件A/B试验，实际摆位也可能不同；不能将坐标差全部量化归于某一份标定。表中的作用是展示输入和目标的实测差异。证据：[当前实录对照](device_context/fixed_vs_yichao_20260910_011920/startup_comparison.json)。

## 底层版本与reference

Fixed配套的是V11 resume42500，66默认镜像HOME_HOLD、V11 common-hold和相对X输入。最近这次实录使用HIT Teacher加`v11-teacher` Arm7 i15000 active；源码确认Teacher/残差只在HIT/POST使用，启动HOME_HOLD仍由student控制。因此不能把启动站稳直接归功于新击球残差。

Yichao配套的是V9 i19000，默认66镜像OUTWARD_HOLD、V9专用HOLD参考、绝对task-anchor X/Y。没有把Fixed的V11完整复制成Yichao底层；两套模型与reference不同，须遵守各自训练合同。

但已经用昨晚Yichao自身有HIT的旧会话对照今天默认版：前300帧reference q/qd/phase、索引/帧号及镜像后参考历史全部相同，最大差值0；原生scheduler两次reset的启动过渡重建也吻合。因此“今天选错reference”目前没有证据。V9待命稳态是0368动作第0帧腿部＋nominal手臂、腰归零后镜像；并不是直接播放索引79/frame54，也不是校准nominal原姿态。

另外核实：Yichao的33个vendored deploy文件与机内原V9字节相同，关节映射和增益/action-scale配置与Fixed一致，原生驱动源码相同；ActiveRunner与历史有击球基准的保存入口AST一致。两套action clip虽分别为10与100，但所查前300帧action绝对值最大分别6.455和6.372，均未到10，裁剪阈值差异不能解释这段启动输出差别。

用同一V9 ONNX回放今天保存输入，q_des最大复现误差`4.77e-7 rad`。在首帧仅替换为旧基准reference历史，输出不变；仅替换task-anchor Y历史，关节目标可变化约0.747 rad。这是单步敏感性证据，不是恢复站稳的闭环证明。完整reference核对见[默认reference报告](YICHAO_DEFAULT_REFERENCE_AUDIT_2026-09-10.md)。

## 应先处理的接入问题

下一步应让Yichao启动时获取当天的共享标定，并记录实际使用的ID、变换和哈希；同次运行保持该份标定一致。先确认66/198实际位姿来源，再评估V9动态待命。不能仅修改Y符号，也不能仅更换HOLD reference。

当天新配置的tracker拟合RMS记录约46 mm，且points-r使用biased版本，旧配置记录约1.7/4.4 mm；这里仅保留配置中的事实，不据此判定重新标定错误。配置适用性仍应结合现场确认，不能将“更新到最新”当成完整输入验收。

本轮没有修改、上传或启停Yichao/Fixed控制程序。Fixed整套代码本地同步已完成，详见[5826文件同步记录](FIXED_STACK_SYNC_2026-09-10.md)。
