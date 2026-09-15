# Yichao V7 model190 + Frozen CBF + V11 i42500

这是与 `yichao_v3_v9`、`yichao_v6r10_v11` 并列的独立部署，工作站目标目录为
`/home/odl/codebase/yichao_v7_v11_adapter`。它不修改旧 Planner、Predictor、共享标定
或机器人 V11 源码。

固定身份为 `v7-model190-cbf-v11-i42500-f052ebf`。Planner handoff 固定在
`f052ebfc51239e83ff58c2067bbb30dd77292e59`，并记录训练审计来源
`1a8e81d0c9ed2a0a027074d3ab2ddfb44e0f4fbc`。远端分支继续前移也不会自动替换模型。

> `model_190` 是 update 190 的实验候选。它没有通过最终 update ≥500 / 32-relay
> 视频门槛，也没有真机可靠性结论；handoff 附带的 8 球材料来自 model150，不能当作
> model190 验收。

## 固定模型与 ABI

| 工件 | ABI | SHA256 |
| --- | --- | --- |
| `v7_planner_actor.onnx` | `[batch,29] → [batch,2]` | `c79054c4…323a7f` |
| `frozen_v7_barrier_risk.onnx` | `[batch,87] + [batch,2] → 3×3 heads / 3 member risks / conservative risk` | `77f68d86…05e54` |
| `model_190.pt` | actor parity source | `c9a568ee…088c` |
| `frozen_cbf_v7.pt` | CBF parity source | `1fa806cb…9064d` |

`models/model_contract.json` 和 `models/handoff_schema.json` 固定完整哈希、字段顺序、
动态 batch、动作顺序及 parity 门槛。运行时只用 ONNX；PT/JIT 只用于离线审计。

Actor 顺序是 `left → table_left → 198 → action[0]`、
`right → table_right → 66 → action[1]`。29D 输入依次为 hitter one-hot、校正并裁到
`0–0.54s` 的 TTS、预测拍速、击球目标、左右 torso XY 各五帧历史。87D CBF 输入
依次为双机五帧 base XY、双机各 29 个归一化关节、击球目标/拍速/TTS、hitter
one-hot。历史按 20ms 插值；球/状态过期、时钟回退或 history gap 都 fail-closed。

## 一拍一次的安全链

严格执行：

```text
Actor raw → clip[-1,1] → ×0.9m → Frozen V7 CBF → 0.50m hard guard → Fixed relay / V11
```

CBF conservative-risk 阈值为 `0.3027352380752564`。nominal 不安全时使用
`51×51 + nominal + home = 2603` 个候选；`1.25` 是归一化动作半径，不是米。
投影按当前 hitter/non-hitter 偏差权重 `100:1` 排序。`[+0.35,-0.35]m` 只作为
模型历史初值和 CBF 最后一个审计候选，不再直接作为真机故障回位目标。没有安全候选
时把该 `shot_id` 永久加入本 session 黑名单，再按下述 Fixed/V3 队形恢复。

独立 hard guard 最终保证 `left_y - right_y ≥ 0.50m`；需要修正时保留 hitter，向外
移动 peer。每个 `shot_id` 只推理一次并锁存双机 Y。命令 X 始终取当帧反馈，V7
不输出也不改变 X。

Fixed relay 继续拥有 reservation、hitter、commit token、HIT/POST_DELAY、return
permission 和失败回退。双侧 staging ACK 验证 session、sequence、锁存 target，且
只接受与当前 stage sequence/session 精确对应的 transport error；旧的
`planner_command_invalid` 不得否决已经受理的新 stage。不增加 6cm 物理到位门槛。

故障回退分两级。若 hitter 明确且双侧反馈新鲜，恢复为 hitter HOME `±0.20m`、peer
OUTWARD `±0.70m`，不会再让双机同时 HOME。若 Controller restart/session/sequence
回退，或任一侧状态缺失、无效、过期，则只发有效 `hold`，保留两侧 V11 当前参考，
不基于不可信状态发起新的横向换位。HIT/POST_DELAY 期间不覆盖原 V11 动作。

真机 relay 恢复 Fixed/V3 的位置语义：ROS 顺序 `table_right / table_left` 下，正常
return/home 为 `[-0.20,+0.20]m`，outward 为 `[-0.70,+0.70]m`，teammate avoidance
为 `[-0.65,+0.65]m`。Actor 仍输出 `±0.90m` 范围，模型历史仍以
`[+0.35,-0.35]m`（model left/right 顺序）初始化；这两组模型值没有被改成 relay 值。

## 模式与入口

- `shadow`：推理、状态、Monitor、录制；不创建 command publisher。
- `transport-shadow`：要求新鲜 V11 shadow 进程证明，只发 inactive `stage`，永不 HIT。
- `active`：完整 Fixed/V11 命令链；工作站和机器人入口保留现场双门槛。

工作站和机器人入口：

```bash
cd /home/odl/codebase/yichao_v7_v11_adapter
bash scripts/start_yichao_workstation.sh
# 等待 WORKSTATION BOOTSTRAP READY，再在另一个终端：
bash scripts/start_yichao_robots.sh
```

机器人入口要求现场输入 `YES`。新鲜 active V11 attestation 出现后，工作站入口才会
从无 command publisher 的 bootstrap shadow 切到 active。详细分阶段操作与停止顺序
见 [DEPLOY.md](DEPLOY.md)。机器人入口在委托原 V11 启动器前，还要求左右 torso topic
各在 3 秒内实际收到一帧；只有 publisher 注册但没有数据时会直接 fail-closed。
该检查在脚本内部显式加载 ROS Noetic，并固定使用
`ROS_MASTER_URI=http://192.168.123.165:11311` 与 `ROS_IP=192.168.123.165`，不依赖
启动终端是否预先 source ROS 环境。

现场 V11 启动器锁定为 SHA256 `27fb30e2…b0a0da`。它对应双机一致的外部 Arm7
runtime `85a68757…62d5f5`，相对旧版只把球刚体 ID 从 `30301` 修正为现场使用的
`30300`；两台机器人均保留修改前备份。V7 仍固定以 `residual_mode=off` 运行，但会在
启动任何 Controller 前显式认证该外部依赖和全部 V11 资产，避免先启动再发现漂移。
共享 V11 `deploy_policy.py` 同时锁定为 `1860b3bc…4097891`，并显式锁定其新增的
Home1000 合同模块 `7f3e7549…78f2b5`；该兼容扩展对本部署使用的 i42500 分支保持原语义。

ROS master 固定为 `http://192.168.123.165:11311`，Planner 状态 topic 为
`/doubles/yichao_v7/status`，Monitor 为 `http://172.16.4.184:8090`。Monitor 只有
录制 start/stop POST，没有机器人控制 API。

## Monitor

主决策卡保持三行×左右两侧六个 Y：Rule-based command、V7 最终 Planner command、
实时位置。Actor/CBF/hard guard、29D/87D、ACK、事件、数据质量、轨迹和原始 JSON
放在下方。

Left / Right Phase 状态流位于机器人卡片之后：实时显示最近 30 秒和预计下一状态；
超过 250ms 显示斜纹 `STALE`，未知 phase 显示 `UNKNOWN`。回放加载完整 phase 索引，
默认看游标附近 30 秒，可切换全程，并明确显示录制中的下一实际状态。状态流只画收到
的 Controller phase，不插值或补造历史；390px 使用内部横向滚动。

## 离线验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/lyz/miniconda3/envs/yichao_sim2sim/bin/python -m pytest -q

/home/lyz/miniconda3/envs/yichao_sim2sim/bin/python \
  tools/replay_v7.py tests/fixtures/v7_two_ball.jsonl \
  output/offline_replay_20260913/replay.jsonl \
  output/offline_replay_20260913/summary.json

python3 tools/verify_delivery_manifest.py
```

确定性 fixture 是合成数据，不代表真机表现。离线 replay 不导入 ROS、不创建 publisher，
用于验证两球各决策一次、重复 shot 锁存、29D/87D、CBF/guard 和 2603 候选时延。
