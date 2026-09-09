# Yichao 今晚部署准备：机载副本已补齐

2026-09-08 18:18 左右重新检查：66、198、工作站二均可 SSH。两台机器人 uptime 约 4 分钟，当时未发现 g1_control 或 policy 进程。用户再次明确现场左边为 198/table_left、右边为 66/table_right。**可连接不等于已交接；机载 shadow 尚未运行，active 仍不可用。**

## 本轮实际完成

两台 `/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z` 已同步完整 relay 源码，完成私有依赖补齐、源码/依赖逐项哈希验证，以及完整 CommandReceiver/ROS bridge 导入。66 先前未完成的环境缺项已补齐。两台被动遥测均用本任务私有 SDK 重新编译，未执行二进制。旧二进制和修改前文件保存在各自 `output/prepare_1788862928003305952/`。

独立环境中读取冻结 V9 i19000 ONNX，以合成零向量验证 `1×1666 → 1×29`：输出有限；20 次单线程 CPU 推理，66 median/max 约 0.856/0.944 ms，198 约 0.802/0.886 ms。不连接 ROS/LCM，不是闭环周期性能或真实动作验证。

两台环境分别记录依赖版本，不能宣称环境逐字节相同；尤其 rospkg/catkin-pkg 来自各自已安装版本。没有安装/升级共享软件，没有修改原 deploy、Predictor、Runtime 或网络配置。

工作站独立目录新增：

- `config/deployment.json`：固定 198 左侧、66 右侧、独立目录、私有遥测 LCM、actor 199＋filter 和有时限 shadow 参数。
- `DoublesTorsoFeedback` 与 `relay_diagnostic_poses.py --input-mode doubles-torso`：消费 Predictor 已变换好的 `/doubles/table_left/torso_pose_origin`、`/doubles/table_right/torso_pose_origin`；保持原位姿、时间戳，不重复标定变换。旧 rigid-1 调试源仍有显式 legacy 选项。相关 8 项测试通过。
- `tools/prepare_shadow_session.py`：生成本次 session 的各主机命令和配置文件，默认不启动任何进程、没有 active 选项。必须传入当前实际 Predictor 节点名。命令含私有姿态转发、两台遥测/机载 shadow 和工作站 relay；这些进程必须在有效期内协调运行，不能把逐个慢慢启动当作可靠执行方法。

生成配置入口（在工作站独立目录；此处 `ACTUAL_PREDICTOR_NODE` 必须替换成当前 ROS 图的实际节点名，不能照抄）：

```bash
./.venv/bin/python -B tools/prepare_shadow_session.py \
  --session yichao_tonight_01 --publisher /ACTUAL_PREDICTOR_NODE
```

输出 `output/sessions/yichao_tonight_01/session.json` 与 `commands.txt`。这只是命令准备；生成成功不代表可进入 default 或接球。不要使用历史节点名或示例名启动。

## 准备阶段当时的未决项（已由后续记录更新）

本轮工作站 ROS 图未发现球预测、monitor 或双机 torso 发布者；有一个 table_left/state 注册项，其是否为存活发布者尚未用该注册项推断。原 Predictor 没有被本任务自动启动/替换。工作站到机器人的 SSH 公钥登录检查失败，未更改认证配置；目前本任务从开发机通过已授权凭据 SSH。

下一步需要明确两台当前测试窗口，恢复原 Predictor/动捕输入，再运行私有 V9 shadow 及真实输入/反馈验收。不能在来源、坐标和时钟未验证时解除 active 门禁。本轮没有启动机载控制程序、没有执行 default/R2 或发电机指令。

证据：[两台准备、备份、制品和 CPU 推理记录](yichao_v3_v9/prepare_1788862928003305952/)。[最新连通性记录](yichao_v3_v9/connectivity_20260908/)中 18:18 的成功结果替代此前超时快照；不能凭此前超时断言电池耗尽。


后续用户已确认双机测试窗口，已完成实际双机 V9 shadow；当前结果与未决项见[晚间实测记录](YICHAO_ONBOARD_SHADOW_2026-09-08_EVENING.md)。不再沿用本页准备阶段“未运行 shadow、无 torso”的状态。
