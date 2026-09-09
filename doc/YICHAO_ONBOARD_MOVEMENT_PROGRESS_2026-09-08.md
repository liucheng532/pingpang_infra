# Yichao 独立执行接口与机载准备进度

2026-09-08 更新。**已完成移动命令受理/完成接口的软件实现及本地、工作站验证；机载 shadow 尚未运行，Yichao active 尚不可用。** 用户最新说明：两台机器人换电池后已开机，朋友将进行测试。此时不启动本任务机载 shadow、policy、g1_control 或控制客户端；保留现场控制权，不处理朋友的进程。

## 代码和部署位置

| 位置 | 实际状态 |
| --- | --- |
| 本地 `yichao_v3_v9/` | 最新实现；98 项测试通过，4.621 秒 |
| 工作站 `/home/odl/codebase/yichao_v3_v9_e14fd5b` | 最新实现已同步；独立 `.venv`；98 项测试通过，3.012 秒 |
| 两台机器人 `/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z` | 已建立独立目录和初版 Python 3.8 依赖副本；两台状态不同，不能作为同一验收版本使用 |
| 198 独立目录 | 已补齐 Debian Python 包；依赖导入与冻结源码校验通过；被动 DDS→LCM 程序已编译，尚未执行 |
| 66 独立目录 | 初版依赖复制完成，但缺失包修复被换电池/SSH 中断；仍需同步最新代码、补齐环境、编译及验证，不能直接启动 |

工作站原 Runtime、Predictor 和两台原 V9 deploy 目录均未修改。未安装共享软件或改动网络、账户、服务。没有启动本任务机器人运行程序。198 的编译使用独立 SDK/LCM 副本；`ldd` 显示 DDS、LCM 动态库均解析到本任务独立目录，标准系统库除外。

## 已实现的接口

- `movement_receiver.py`：专用 `yichao-v9-movement-v1` 命令，仅支持 MOVE；整包验证后在 scheduler 副本上试应用，确认目标锁存后提交。反馈包含 session、robot、sequence、shot、token、实际应用目标及 accepted/rejected/completed。禁止 HIT；不会借用旧 bridge 提前增加的 sequence 伪造成功。
- `clock_bounds.py`：四时间戳交换，记录工作站/机器人 monotonic 偏移区间。交换不确定度超过 20 毫秒或租期超过 1 秒时拒绝新命令；不拿收到消息的时间替换命令生成时间。
- `movement_client.py`：单机、一次移动事务；使用 actor 199 与安全过滤器，对静止同伴重新评分。仅明确应用反馈更新目标记忆。当前试运行位移上限 5 厘米，超出则拒绝，未裁剪模型工作区。它不是完整双打 relay。
- `onboard_movement.py`：独立 ROS bridge，要求 command/state 重映射到 `/yichao_v3_v9/<session>/<role>/…`。附带关节、IMU 与 torso 接收龄；动捕原始采样龄尚未证明，`input_contract_accepted=false`。
- `run_onboard_movement.py`：冻结 V9 student/scheduler/mirror 的机载 shadow 入口。CPU、单线程推理、最长 60 秒；独立遥测 LCM `239.255.77.<robot>:7767`，LCM 包装器禁止任何发布。发现已有 `g1_control`、deploy policy 或另一机载 shadow 时拒绝启动。`--mode active` 在建立 ROS/LCM 连接前报错。
- `passive_v9_telemetry.cpp`：只订阅 DDS `rt/lowstate`、`rt/secondary_imu`；CRC/有限值检查后，向指定私有 LCM 发布关节与 IMU 状态，45 秒退出。没有 LowCmd 类型或电机指令订阅。源码和编译产物的存在不等于真实遥测验收。
- `capture_onboard_shadow.py`：工作站只读采集独立 state 和现有球预测；双机数据满足诊断条件后运行 V3 特征/推理。单机采集不补零、不冒充双机接口通过。它不构造任何 ROS 发布者。

本轮修复了完成反馈被重发覆盖、球预测消失后事务超时不再检查、拒绝消息在后续周期重新尝试应用等问题。完成须连续稳定位置/速度证据，不能由 HOLD 标签或单独 ACK 推定；停止发布也不表示物理停止。

## 验证和证据

[证据目录](yichao_v3_v9/onboard_movement_20260908/)包含本地/工作站测试输出、工作站发布清单、初次机载复制结果、换电池中断记录及 198 环境/源码清单和原生编译记录。98 项测试覆盖已有 V3/安全过滤/离线 relay 与新增镜像移动应用、字段拒绝、去重、稳定完成、时钟边界、LCM 发布禁止、进程冲突以及机载无效源龄拒绝。

机载 Python 3.8 不能导入 Planner 包部分运行时 `|` 类型别名。适配代码已把参考 `RealInputAdapter` 的导入移入工作站输入构造函数；机载仅导入通用校验和冻结 scheduler，未修改 Planner 的冻结源码。环境复制已处理 Debian 包缺失 RECORD/SOURCES 或空 top_level 的情况。

真实关节→机载 V9 observation→student→私有 state 的整条运行尚未验证；本轮没有产生真实 V3 决策、真实命令 ACK 或电机输出。此前动捕、关节只读录制的结果仍按各自证据范围使用。

被动遥测携带的是机器人本机 DDS 回调的 monotonic raw 时间，不能解释为硬件传感器采样时间；它能约束本机接收后的滞留时间，尚不能证明 DDS 之前的样本年龄。这与未证明的动捕源龄一起阻止真实输入验收，不能通过 `sensors_recent` 单独解除 active 门禁。

## 后续顺序与回退

1. 朋友测试期间只继续本地/工作站独立开发；不启动机载程序。取得本任务设备窗口后重新核对进程、负载、ROS 图、模型及现场模式，不能沿用换电池前的快照。
2. 两台同步工作站的最新适配源码，并更新各自源码清单。尤其 66 需用修正后的 `provision_onboard_shadow.py --complete-missing` 完成私有依赖，核对旧版本不被替换；使用 `build_passive_telemetry.py` 在私有 SDK 副本中编译。以上是准备步骤，当前没有自动补跑。
3. 先启动有时限的私有 torso relay、被动遥测和机载 shadow，验证 1666→29 student 推理及双机私有 state；该步骤不启动 g1_control。完成后才接入只读 V3 诊断和专用移动反馈验证。
4. 仍须核清共同坐标/朝向、动捕与预测源龄、历史采样、初始目标记忆，以及完整 HIT/RETURN 交接和反馈。当前单次 MOVE 接口不能替代完整 relay，不能解除 active 门禁。

工作站本轮修改前备份：`output/movement_bridge_20260908/before.tar.gz` 与 `before_final.tar.gz`，发布清单随备份保存。机器人是新增目录；没有覆盖原 V9。当前没有本任务机载运行进程需要停止。后续退出只处理记录的本任务确切 PID；不自动重启原方案，不把目录回退当作撤销已执行动作。
