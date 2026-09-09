# 朋友测试期间的只读输入核对

2026-09-08：**已获得同期双机状态与有效球预测记录；当前现场运行的是 V11 resume42500，并非本任务指定的 V9 i19000。** 没有切换现场版本、修改原文件、启停他人程序或发布控制命令。

## 现场进程身份

只读 `/proc` 和运行参数核对：66 的 g1_control/policy PID 为 5155/5320，198 为 8894/9013。两台 policy 均在 `doubles-v11-r2i299-resume42500-dual-runtime`，加载 `student_v11_r2i299_commonhold_1666_resume42500.onnx`，文件 SHA256 为 `d7be604c882307f6951c4649607dd6a07e6dbbd31bd3d80a481920ff5f762eaa`。这是进程参数及磁盘文件证据，不是对运行内存中模型的读取。

两台都发布 `v9-robot-state-v1` 格式。因此 topic/schema 名字不能证明底层版本为 V9。原 Fixed Planner `/doubles_v9_real_fixed_relay` 持有两路控制 topic；本任务未建立控制发布者。

## 20 秒只读录制

工作站独立环境以 nice 19、单线程 CPU 运行有时限的 input diagnostic，仅读取两路 state 和 ball prediction。20 秒后正常退出。

| 输入 | 收到回调 | 留存原始记录 | 回调频率 |
| --- | ---: | ---: | ---: |
| 66 / table_right state | 1000 | 627 | 约 50 Hz |
| 198 / table_left state | 1000 | 627 | 约 50 Hz |
| doubles ball prediction | 1592 | 409 | 约 79.6 Hz |

采集器按至少 20 ms 间隔留存，回调抖动会使保留率低于 50 Hz；627 条不是状态丢帧数量。采集队列丢弃与错误发布者均为 0。回调最大间隔分别约 25.0、23.7、48.0 ms；这些是接收端统计，不是端到端传感器延迟。

离线字段检查：两路各 627 条状态通过 wire 格式检查，ball 的 409 条留存中 39 条为有效预测并通过格式检查，370 条由发布者声明 invalid。没有补零、修改有效标记或替换源时间。

实时诊断推理为 0：首次状态的 `transport_error=planner_command_invalid` 触发现有严格诊断门禁。冻结 V9 bridge 中该标记表示上条命令 valid=false，不等于传感器必然失效。198 的记录还含 27 次 `command_rejected:Deployment scheduler supports lateral-Y base targets only.`；仅凭该消息不能推定所有命令字段均未生效，也不能认作 Yichao ACK。本轮没有放宽门禁或修改现场 bridge。

## 证据及下一步边界

[证据目录](yichao_v3_v9/friend_test_readonly_20260908/)包含原始记录、采集执行输出、运行参数与模型/bridge 哈希、bridge 只读副本、wire 审计和退出后的 ROS 图。原始文件约 4.3 MB。诊断节点退出后再次检查，`yichao_input_diagnostic_*` 节点为 0，两路控制发布者仍是原 Fixed Planner。

这关闭了“现场完全没有双机 state/有效球预测”的历史缺项，但没有证明真实输入时钟/共同坐标已验收，也不构成 V9 student 或 Yichao 闭环证据。本任务仍使用 V9 整套制品，不能直接把 learned planner 插入朋友的 V11 控制链路。

目前需要朋友测试结束及两台设备交接，才能继续本任务机载目录补齐、V9 shadow 和真实接口验收。旧进程的存在已实测，不能将机器人开机或 ROS 数据可读解释为可接管。本任务未启动的机载程序无需停止；不处理上述他人 PID。
