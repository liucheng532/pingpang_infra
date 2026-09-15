# V9 Doubles Student 部署 Handoff

更新日期：2026-08-18

## 1. 结论

当前默认部署模型是 **V9 timed-handoff 1666 student `model_19000`**，不是训练任务当前仍在生成的最新 checkpoint，也不是1696版本。

## 左手镜像部署

球台右侧、世界 Y 为负、左手持拍的机器人复用同一个 canonical V9 ONNX。启动时必须增加
`--mirror-left-hand`；该模式会同时镜像物理 reference、完整1666维observation和按绝对关节
目标变换的29维action。不能只对normalized action交换左右通道。

首次验证必须使用 `--shadow --record-dir ...`。recorder 会同时保存物理observation/action和送入
canonical ONNX的observation/action，便于与仿真中 `left` 机器人链路逐帧对齐。

- 部署目录：`/home/unitree/haoran/doubles-student-1666-deploy`
- 部署分支：`feature/doubles-student-1666-deploy`
- 当前部署代码：`f72d4ae`
- V9默认模型：
  `policy/v9_model19000/student_v9_m14500_timedhandoff_1666_model19000.onnx`
- V9 student checkpoint SHA256：
  `2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a`
- V9 ONNX SHA256：
  `7e7e26d71895eb900c7b95fa2cd2fb69cbe8207586869390138c6f758848cae7`
- 输入/输出：`1666 -> 29`

上一版部署模型是 **m73000六状态1666 student `model_23000`**。项目目录和实验记录通常把这一代称为“V6 student”，但 checkpoint metadata 本身没有保存字符串 `V6`，因此本文统一写作 **旧V6/m73000**。

## 2. 两代模型身份

| 项目 | 旧V6/m73000部署 | 当前V9部署 |
|---|---|---|
| Student checkpoint | `model_23000.pt` | `model_19000.pt` |
| Student iteration | 23000 | 19000 |
| Student SHA256 | `f2d31b802a37c014971bf83a0b9d1a61e2b8727b42ffbed3dd153b7e4d67127d` | `2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a` |
| ONNX文件 | `student_m73000_hold6_1666_model23000.onnx` | `student_v9_m14500_timedhandoff_1666_model19000.onnx` |
| ONNX SHA256 | `43226d95a2120e85b949f0c3318b64c1d2819666d060f53be8a84bc4d85b687c` | `7e7e26d71895eb900c7b95fa2cd2fb69cbe8207586869390138c6f758848cae7` |
| Hit teacher | 同一固定V00 ONNX | 同一固定V00 ONNX |
| Hit teacher SHA256 | `5a4e650813f227f704ae2040df73ece8da11dae1746bfc8b6e982c68e9efc94c` | 相同 |
| Move teacher | `model_73000`，旧V6/m73000代 | V9 wide-home arm-fix `model_14500` |
| Move teacher SHA256 | `9255177c1ae6eced0a88547ea0d362cced22b3fbda2bd044221f43846345f260` | `8716d2e5865674f210e1261a18c5ed2b370b7de2ce641e6f10387f416ae11296` |
| Move teacher contract | checkpoint未记录contract字符串 | `v9_widehome_armfix_model14500` |
| Student输入 | 1666 | 1666，完全相同ABI |
| Phase | 六状态 | 六状态，顺序相同 |
| Action delay训练 | reset随机0～8 physics steps | 相同 |

旧V6/m73000 student正式实验：

- run：`doubles-distill-m73000-hold6-d08-1666-s42-i200000-0809`
- DLC：`dlc1ote22ywb787x`
- W&B：`7hyitqly`
- 训练代码：`fe36038fc13666e2f76ef1c311ce9996e01afbc1`

当前V9 student正式实验：

- run：`doubles-distill-v9m14500-timedhandoff-pd0205-hold6-d08-1666-resume10500-s42-i200000-0815-r3`
- DLC：`dlc1mld4wanu7mgf`
- W&B：`sirhuy7p`
- 训练代码：`0514b23c9f85e13a5c6a5e7420879fde3c32e2b9`
- `model_19000`来自该run；该run从前一条V9 gated student的`model_10500`恢复完整训练状态后，切换到timed-handoff继续训练。

## 3. 保持不变的接口

两代student可以共用硬件控制和1666维输入ABI：

```text
strike        [0:10]
target_vel    [10:40]
racket        [40:70]
target_base   [70:90]
task_anchor   [90:120]
orientation   [120:180]
base_ang_vel  [180:210]
joint_pos     [210:500]
joint_vel     [500:790]
actions       [790:1080]
command       [1080:1660]
phase         [1660:1666]
```

- history：每个term十帧，term-major，oldest-to-newest。
- phase只使用当前帧，不进入history。
- phase顺序：
  `HIT / POST_DELAY / OUTWARD / OUTWARD_HOLD / RETURN / HOME_HOLD`。
- 输出为29维raw action。
- `0302_combined` hit数据、`0718-move-160-80hz` move数据、关节映射、PD、action scale和LCM ABI不变。
- 部署时不加载两个teacher，只加载student ONNX、motion数据和状态机。

## 4. V9相对旧V6/m73000的主要变化

### 4.1 Move teacher升级

旧student模仿`model_73000` move teacher。V9 student改为模仿V9 `model_14500`，其主要训练目标是：

- HOME Y覆盖从旧student环境的约`0.30～0.40 m`扩展到`0.00～0.40 m`。
- TARGET_HOLD使用稳定姿态reference，不再把0718 outward末帧的不对称腿部姿态直接作为长期HOLD目标。
- HOLD双臂目标使用G1 nominal default pose。
- 腰部HOLD目标为neutral yaw/roll/pitch。
- 增加关键肩、肘关节的强跟踪和worst-joint惩罚，避免某个手臂关节的异常被group mean掩盖。
- V8/V9 teacher训练中加入长短混合HOLD、直立/高度/膝关节约束以及HOLD扰动恢复；V9进一步覆盖更宽home范围并加强手臂修复。

### 4.2 OUTWARD_HOLD reference改变

旧V6/m73000部署的OUTWARD_HOLD reference：

- 腿：0718 outward motion最后一帧。
- 双臂：旧0302 ready pose。
- 腰：neutral。
- qdot：0。

当前V9部署的OUTWARD_HOLD reference：

- 腿：`0302_combined-0368:v0`第0帧的稳定下肢姿态。
- 双臂：G1 nominal default pose。
- 腰：neutral。
- qdot：0。
- 从outward末帧到稳定HOLD reference做`0.30 s` smooth transition。

这是两代部署最重要的reference语义差异。

### 4.3 HIT到OUTWARD的触发改变

旧V6/m73000 student训练使用readiness gate：

```text
strike结束
-> POST_DELAY至少0.20 s
-> 检查姿态、线速度、角速度、双脚接触
-> 连续ready 0.10 s
-> OUTWARD
```

当前V9 timed-handoff student训练不使用机器人自身ready状态决定移动时机：

```text
strike结束
-> 独立采样POST_DELAY 0.20～0.50 s
-> 时间到直接OUTWARD
```

目的：最终双打移动时机由planner/队友事件决定，而不是机器人等自己站稳后才移动。

当前真机scheduler固定使用`POST_DELAY=0.20 s`，位于V9训练分布下界；代码不会执行旧readiness gate。

### 4.4 HOLD时长与恢复场景改变

旧V6/m73000 student主要训练：

- 80% HOLD `0.20～0.50 s`。
- 20% HOLD `0.50～1.00 s`。

当前V9 student训练：

- 56% HOLD `0.20～0.50 s`。
- 14% HOLD `0.50～1.00 s`。
- 30% HOLD `1.00～5.00 s`。
- 30% TARGET_HOLD样本施加一次小扰动，学习HOLD闭环恢复。

当前真机scheduler固定`OUTWARD_HOLD=3.0 s`，属于V9长HOLD训练范围，但不是训练分布的多数样本。

### 4.5 Reset和下一拍场景更丰富

V9 student训练场景包含：

- 70%正常循环。
- 15% cold HOME启动。
- 15% locomotion phase preemption。
- cold HOME等待和无下一拍样本。
- OUTWARD、OUTWARD_HOLD、RETURN中的新球抢占样本。

当前部署检测到一条重新armed的有效TTS后，只要当前不在`HIT/POST_DELAY`，就允许从OUTWARD、OUTWARD_HOLD、RETURN或HOME_HOLD直接进入下一次HIT。这与旧版“只有HOME_HOLD接受新球”不同。

## 5. 当前部署特有设置

当前`f72d4ae`部署代码还包含以下运行时设置：

- 默认V9冻结模型为`model_19000`，不会跟随DLC的`model_latest.pt`。
- `POST_DELAY=0.20 s`。
- `OUTWARD_HOLD=3.0 s`。
- outward target Y默认`0.9125 m`。
- home Y在scheduler reset时从重建的pelvis Y锁定。
- move motion index 47被禁用，不参与nearest-motion选择。
- pelvis位置由`/torso_pose_origin`和实时腰关节通过deploy侧FK重建。
- 新球允许抢占locomotion状态。

## 6. 为什么当前仍冻结在model_19000

V9 timed-handoff 1666训练任务仍在继续，已产生比`model_19000`更新的checkpoint；但“iteration更新”不代表综合任务表现一定更好。

对`model_19000`和`model_70500`做过同场景仿真比较：

- `model_70500`的imitation MAE、跑位Y误差和击球位置误差部分改善。
- 击球方向误差和strict success有退化。
- delay=8时没有证明跑位或防摔有明确综合优势。
- 两者在该固定测试中均没有`bad_robot_ori`。

因此`model_70500`没有被批准替换真机模型。部署继续冻结`model_19000`，后续应对多个中间checkpoint做统一seed/delay/任务指标选模，不能直接部署`model_latest.pt`。

## 7. 启动与确认

默认启动：

```bash
cd /home/unitree/haoran/doubles-student-1666-deploy/g1_gym_deploy
python scripts/deploy_policy.py
```

启动时必须确认日志/sidecar显示：

```text
input_dim = 1666
output_dim = 29
checkpoint_sha256 = 2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a
move_teacher_contract = v9_widehome_armfix_model14500
phase_order = HIT, POST_DELAY, OUTWARD, OUTWARD_HOLD, RETURN, HOME_HOLD
```

文件校验：

```bash
sha256sum \
  /home/unitree/haoran/doubles-student-1666-deploy/policy/v9_model19000/student_v9_m14500_timedhandoff_1666_model19000.onnx
```

预期ONNX SHA256：

```text
7e7e26d71895eb900c7b95fa2cd2fb69cbe8207586869390138c6f758848cae7
```

## 8. 回滚注意事项

不要只把`--policy`指向旧V6/m73000 ONNX后继续使用当前V9 scheduler。

原因：两代student虽然都是1666维，但OUTWARD_HOLD command/reference语义不同。旧student期望“0718 outward末帧腿部＋旧ready arms”，当前scheduler会提供“0302稳定腿部＋nominal arms”。只替换ONNX会产生训练—部署reference错配。

需要回滚时应整体使用旧部署代码/状态机和旧ONNX：

- 旧部署tag：`backup/doubles-student-1666-deploy-20260810`
- 旧部署commit：`d82500e`
- 旧student checkpoint SHA256：
  `f2d31b802a37c014971bf83a0b9d1a61e2b8727b42ffbed3dd153b7e4d67127d`
- 旧ONNX SHA256：
  `43226d95a2120e85b949f0c3318b64c1d2819666d060f53be8a84bc4d85b687c`

建议另建旧版部署目录进行回滚，不要在当前V9目录里混用两代scheduler、sidecar和ONNX。

## 9. 交接检查清单

- 确认使用的是1666，不是1696。
- 确认ONNX和sidecar成对存在，hash完全一致。
- 确认0302/0718 manifest hash通过启动检查。
- 确认启动日志显示V9 `model_19000`，而不是`model_latest`或其他iteration。
- 先用`--shadow --record-dir ...`检查TTS、pelvis、phase、reference和action。
- Active测试时确认phase按六状态切换，POST_DELAY约0.20秒。
- 检查OUTWARD_HOLD进入后使用stable legs、nominal arms和neutral waist。
- 检查新球在locomotion阶段能够抢占并进入HIT。
- 出现异常时保留recorder目录，并记录代码commit、ONNX SHA256和启动命令。
- 回滚必须同时回滚scheduler和模型，禁止只换ONNX。
