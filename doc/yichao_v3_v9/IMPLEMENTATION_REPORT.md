# Yichao V3 → V9 离线实施与验收记录

日期：2026-09-06。

**工作站交付更新：** 已在 `/home/odl/codebase/yichao_v3_v9_e14fd5b` 建立实际代码、固定资产、独立 `.venv` 和 `run_workstation_offline.sh`。新增 peer-clear/HIT 成对提交、V3 动态 outward/runway 投影和显式 RETURN 完成链。工作站 21 项测试通过；249 帧合成回放完成两拍，最大处理耗时约 3.05 ms。入口及边界见 [工作站说明](../../yichao_v3_v9/WORKSTATION_START.md)，记录见 [workstation_delivery.json](workstation_delivery.json)。下文初轮 169 帧/19 项结果为历史快照，不代表更新后的 relay 仍缺少 peer-clear/RETURN。
独立实现：[`yichao_v3_v9`](../../yichao_v3_v9/README.md)。

已交付可重复运行的离线接口验证包：固定代码/模型/动作副本、36D/85D 输入入口、真实 ONNX 推理和安全筛选、保守 relay、原版 V9 scheduler/mirror 执行器、独立命令/反馈契约及测试。**数值与离线接口测试通过；完整 V3 relay 语义、真实输入和在线阶段尚未验收。** 不将保守测试框架的两拍完成等同于训练 relay、低层物理闭环或真机成功。

用户说明现场正在运行真机测试，允许只读访问设备。此次只做本地开发及 SSH/SFTP 只读资产核对/复制；没有启动现场测试、ROS 节点、policy 或控制器，没有发布指令、安装现场软件、更改现场配置或启停他人进程。原本地 Planner、deploy、controller 工作树均保持干净。

## 1. 固定资产与证据等级

| 对象 | 本次结果 | 证据边界 |
| --- | --- | --- |
| Planner | 从 `e14fd5ba3daa6327ac842d53fe5c4292b04e5716` 导出独立快照；交付 SHA256SUMS 验证 | 本地 Git 对象与交付文件实测 |
| deploy | 从 `347890eb050a9b7f0856f311449d68f6411b6d92` 导出纯计算依赖 | 不覆盖两机现场树 |
| 66 | 现场 HEAD `347890e` | 只读快照；不证明设备空闲 |
| 198 | 现场 HEAD `c207435`，启动脚本与 66 不同 | 本包以 clean `347890e` 为代码基线，不迁入启动脚本差异 |
| scheduler/mirror/bridge | 两机关键文件哈希相同，且与本包 clean deploy 对应文件相同 | 内容一致，不代表当前运行进程加载这些文件 |
| V9 ONNX | 两机均 `7e7e26d7…48cae7`，与手册一致 | 完整 SHA256 在清单和原始证据 |
| V9 sidecar | 两机均 `fd6d14cb…78da89`，`1666 → 29`、i19000、phase 顺序匹配 | 模型 ABI/metadata 验证，不是实际控制输出验收 |
| V9 checkpoint | 交付内 `2c70e278…358eb8a`，匹配 sidecar | 本次使用冻结本地交付 PT；不宣称再次读取现场 PT |
| 动作库 | 每台核对 155 移动 +441 击球 NPZ，以及两个索引 | 全部数组逐值一致；下述封装差异不隐藏 |
| 环境 | Python 3.11，CPU ORT 1.23.2、Torch 2.9.1+cpu、NumPy 1.26.0，独立 site-packages | 本机依赖；未在工作站安装或性能验收 |

原始证据：[`remote_assets.json`](remote_assets.json)、[`motion_array_audit.json`](motion_array_audit.json)、[`assets.lock.json`](../../yichao_v3_v9/config/assets.lock.json)。远程读取结束后未保留后台采集进程。

两台均核对 598 个文件。441 个击球 NPZ 和两个索引文件逐字节匹配；155 个移动 NPZ 的文件 SHA256 不同，但字段集合、数组形状和所有数值逐值一致。规范化数组摘要对每个字段固定连续布局和小端字节序，同时保留 dtype/shape；另做 `array_equal`，避免只看容器哈希下结论。未发现数值差异。远端副本单独保存，本包执行用固定本地主副本。

计划中工作站 Predictor/Runtime 的 commit、未提交差异、网卡和 ROS 图属于此前核查叙述。本次没有将其重新记作实测，更没有迁入现场 Runtime 补丁。完整运行参数、二进制构建来源、有效标定、网络拓扑和当前发布者仍须在未来只读核查中明确。

## 2. 特征与安全管线

机器可读接口表：[`interface.json`](../../yichao_v3_v9/config/interface.json)，包含 36+85 个逐索引字段的来源、变换、单位、时间、有效性与缺失策略。

- 槽位固定为训练负 Y/left → 66/table_right/镜像；正 Y/right → 198/table_left/canonical。仅在合成共同坐标中使用，真实坐标变换未放行。
- 36D：由当前球状态及加速度反推五个历史点，减对应双机历史位置；按 `[3,1.5,1.5]` 缩放和 ±3 裁剪；拼接当前 base XY 和确认的上次目标。不重复执行 actor 内经验归一化。
- 85D：按机器人优先顺序拼接五点 base XY、各 29 个归一化关节、击球点、**拍速**和 TTS。参考测试直接调用冻结 `v0.py` 的特征函数，避免仅比较两份自行改写的实现。
- 夹具节拍 0.02 s、关节 soft limits ±2 rad、部署 LAB 关节序仅是显式测试输入。未将它们当作真实训练参数。初始目标记忆来自合成 scheduler bootstrap；真实冷启动和重启记忆尚未定义。
- 源时间、接收时间和时钟域分别保留；反馈限制 0.25 s、预测限制 0.30 s；按源龄修正 TTS，不以收到消息的时间覆盖旧源时间。跨机 monotonic 不能直接比较。当前只接受单一 replay 时钟域。
- 候选为 nominal、7×7 网格及 home，共 51 个；网格半径 0.75，阈值从冻结制品提取并与原始 PT 对照为 `0.3905482217669487`。
- 筛选按距离平方 + `1e-3 × risk`；目标解析间距至少 0.45 m，投影后重新评分。无安全候选锁存故障，home 不被视为必然安全。

所有真实记录无条件拒绝。接入现场前须补齐共同坐标/朝向、训练 soft limits 与关节序、历史采样和初始化、跨机时钟转换及误差界。85D 模型输入不要求 FK，不代表训练协议中的额外 FK/碰撞保护可以省略。

## 3. 命令与 V9 执行验证

独立契约见 [`command.schema.json`](../../yichao_v3_v9/config/command.schema.json) 和 [`lifecycle.json`](../../yichao_v3_v9/config/lifecycle.json)。离线消息不同于现场 `v9-planner-command-v1`，不能直接发入现场 topic。

完整校验 session、sequence、shot/token、目标、时效、HIT 点/拍速/球速/返回目标后，才调用 scheduler 副本。所有副本成功后，离线批次才生效。这样覆盖现有 bridge 在启动 HIT 后才发现其他字段非法的部分应用问题；**没有修改原 bridge，也不声称真实双机事务可回滚**。

目标记忆只从匹配命令身份的有限值 ACK 更新。两机分别记录已受理和完成；其他会话 ACK、phase 标签、无目标的“ready”或旧 `last_applied_sequence` 不算确认。进入 RETURN 只算运动状态，到位确认要求目标误差、速度和稳定时间。

本次发现并复现的 V9 行为：

| 场景 | 原版 scheduler 实测 | 适配处理 |
| --- | --- | --- |
| 位移 ≤0.0001 m | `set_external_base_target` 可直接返回 | 核对目标是否实际锁存；不把函数返回视作到位 |
| 同 OUTWARD/RETURN phase 改目标 | 目标更新，原 reference index/step 保留 | 日志区分目标更新和 reference 切换；不声称重新规划动作 |
| 方向反转 | 按新方向选动作，可能换 phase | 验证选择、禁用索引和镜像对称；没有改 scheduler |
| HIT/POST_DELAY 普通移动请求 | 拒绝，不能靠普通 base target 中止击球 | 不启用 safety_override 绕过 phase 保护 |
| 显式 RETURN | 运动类别依赖 V9 动作选择 | 仅接受 RETURN/HOME_HOLD 语义；受理和到位分开 |
| 不再发送命令 | 已有 HIT 继续经过 POST_DELAY、OUTWARD | FAULT 表示禁新命令，不能解释为物理停止 |
| HIT TTS | scheduler 裁剪至最高 0.54 s | ACK 同时记录实际 scheduler strike time，保留请求值 |

### 初轮 relay 缺口与后续实现

初轮保守框架等两机到位后才 commit，使用固定 ±0.9125 的 post-HIT outward，保持 peer 的准备目标，以 hitter 的 outward 完成作为合成 handoff，再开始下一拍准备。这验证了目标锁存、交替身份及 V9 钩子的调用关系，**并未完整复现训练端 relay**。

冻结 `isaac_rl.py` 的 `_commit_many/_commit_one` 还会对 peer 发 outward clear，并根据下一 hitter 设置返回对；`_physical_outward_target`、`_project_controller_motion_target` 和 `_skill_mask` 检查动作可表达性、runway、阶段及几何/TTC。V9 当前钩子与精确训练 Controller dirty patch 的关系尚未逐项验证。不能把固定 relay 的 hold/no-op、简单两目标或这个保守框架当成完整替代。

后续本地实现应先从上述源码导出 prepare/commit/peer-clear/RETURN 的逐阶段参考向量及动作类别，核实对应训练 Controller 参数，再扩展独立 supervisor 和明确 ACK。V9 不可表达的目标/阶段必须拒绝并报告，不自动改 scheduler、缩小 actor 工作区或套用 V10/V11 reference。后续本轮已复用冻结纯投影方法，补齐 peer-clear、动态 outward 和 RETURN 完成路径，并在工作站通过测试。在线接线仍保持关闭：真实坐标/时钟/soft limits、几何/TTC 保护和精确 dirty patch 尚未验收。

## 4. 验证结果与可复现性

本次最终 **19 项测试通过**。带版本身份的两份 CLI 时序日志与摘要见 [`release_index.json`](release_index.json)。

完整用例及结果：[`test_results.json`](test_results.json)。执行命令见包 README。

已覆盖：190/199 actor 和 filter 三类输出的 batch 1/7/32 对照；36D/85D 源码参考向量；阈值边界、网格介入、无安全候选、解析投影；缺失/错误维度/NaN/Inf/四元数/phase/源龄/未来时间/乱序/时钟域；命令预校验、重发、旧 token/session、同 shot 重复 HIT、单机拒绝、部分反馈、ACK 伪成功、超时和处理预算；小位移、反转、同 phase 目标更新、镜像、RETURN 到位及既有运动继续执行。

模型对照最大绝对误差约 `1.0431e-6`，低于交付 `2e-5` 容差。低层 ONNX 的 `1×1666 → 1×29` 有限值输出仅作 ABI smoke，零输入不是真实有效机器人状态。

169 帧两拍夹具中，actor 199 和 190 的独立 CLI 回放均到达框架 COMPLETE，198/66 各一次 HIT，无无效输入和超预算丢弃。夹具由 actor 199 框架生成，190 在同一固定骨盆轨迹上对照，不是公平仿真质量比较，也不表示199真机更优。

单帧端到端处理时间包含特征、推理（仅每拍一次）、relay 和 scheduler 包装；本机样本最大约 4–6 ms，具体数值以 JSON 结果为准。超20 ms的新决策被拒绝；不积队列补发。文件回放没有传输排队，`queue_wait_ms=0` 是入口属性，不能用它证明50 Hz现场实时性；尚未做工作站算力、调度抖动或真实丢帧验收。

## 5. 后续门槛与回滚

当前不能进入完整 Planner shadow：真实输入证据门槛未通过，完整 relay 语义仍未验收，且此前所述发布者缺失情况没有在本次重新确认。没有自动启动或替换 Predictor，也没有为拿到输入占用现场运行资源。

未来只读 observer 应先登记唯一节点名、订阅源、时钟来源及 CPU/内存/磁盘预算；默认仅独立日志输出。如需要诊断 topic，限定 `/yichao_v3_v9/<run_id>/…`，不能进入控制链。现场 phase 只能说明他人控制器的状态。

机载 shadow、active 和控制权交接仍未开展。不得仅靠独立目录或 topic 并发控制同一机器人。缺失的 q_des rate limit、姿态/高度及 tracking-error 保护没有在本任务中被假定为已存在。精确训练 `distill_commands.py` dirty patch 仍缺，不能声称精确复现交付仿真。

本次产物回滚只涉及独立本地目录，不需恢复现场任何内容。未来发布/回滚必须把 Planner、filter、scheduler、sidecar、ONNX、mirror 和动作库作为整套。不得自动重启他人基线或用撤回消息解释已经发生的机器人动作。
