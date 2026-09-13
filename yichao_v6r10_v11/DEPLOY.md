# V6R10 / V11 独立部署操作

工作站目标目录固定为：

```text
/home/odl/codebase/yichao_v6r10_v11_adapter
```

两个入口不创建 roscore，不停止冲突进程，不覆盖旧目录。检测到 Predictor/Planner
占用、command publisher 或 8090 占用时会拒绝启动。

## 推荐：两个现场入口

正常上机只需两个终端，不需要手工切换 shadow。先在终端一运行：

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
bash scripts/start_yichao_workstation.sh
```

看到 `WORKSTATION BOOTSTRAP READY` 后，在终端二运行：

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
bash scripts/start_yichao_robots.sh
```

机器人入口要求现场输入一次 `YES`。V11 active 启动及实时 attestation 通过后，
终端一会自动停止无命令 shadow 并切换为 V6R10 active；看到
`V6R10 WORKSTATION ACTIVE READY`、核对 Monitor 8090 后，再按既有 V11 流程操作
R2。该自动接力保留原有 torso、hash、进程身份和现场安全门槛，没有修改机器人文件。

停止时先工作站、后机器人：

```bash
bash scripts/start_yichao_workstation.sh stop
bash scripts/start_yichao_robots.sh stop
```

下面各节是诊断和分阶段验收使用的底层入口，正常上机无需手工执行。

## 1. 只读检查

```bash
cd /home/odl/codebase/yichao_v6r10_v11_adapter
./scripts/start_v6r10_workstation.sh check shadow
./scripts/start_v6r10_workstation.sh dry-run shadow
./scripts/start_v11_robots.sh dry-run shadow
```

核对 `ROS_MASTER_URI=http://192.168.123.165:11311`、8090 空闲、双 command topic
无 publisher、旧 V9/Fixed 进程未占用。`DEPLOY_MANIFEST.json` 可用下式复核：

```bash
python3 tools/verify_delivery_manifest.py
```

## 2. Workstation shadow

此模式没有 command publisher，可先启动原 Predictor、新 Planner 和新 Monitor：

```bash
./scripts/start_v6r10_workstation.sh start shadow
```

在另一终端查看 `status shadow`、8090 和 session 日志。确认状态 topic 存在且
`/doubles/table_left/command`、`/doubles/table_right/command` 仍无新 publisher。
完成后仅停止本入口记录的 owner：

```bash
./scripts/start_v6r10_workstation.sh stop
./scripts/start_v6r10_workstation.sh status
```

## 3. V11 Controller shadow 与 transport-shadow

只有原 V11 启动器的无占用、torso topic、模型/sidecar/动作资产和双机网络预检全部
通过时，才可启动 Controller shadow：

```bash
./scripts/start_v11_robots.sh check shadow
./scripts/start_v11_robots.sh start shadow
```

入口固定转交原启动器参数 `shadow normal off 1.0 v11-teacher`。随后生成的证明包含
双机 boot ID、PID/starttime/cwd/exe/argv、主 student、checkpoint、common hold、
HIT teacher、motion manifest、V11 teacher、bridge、scheduler、mirror 和 native
binary 的 hash。

停止 workstation shadow 后，以同一批仍存活的 Controller shadow 启动：

```bash
./scripts/start_v6r10_workstation.sh start transport-shadow
```

该入口会重新现场生成并校验不超过 120 秒的进程证明。它只发布 inactive stage，
验证两侧 ACK，不会发布 HIT。验证结束后先停止工作站入口，再通过原委托入口停止
双机 shadow，并复核进程与 command publisher：

```bash
./scripts/start_v6r10_workstation.sh stop
./scripts/start_v11_robots.sh stop
```

网络、torso、资产、进程或 ACK 任一门槛失败时保留失败证据，不绕过、不改旧启动器。

## 4. Active 底层入口

只有用户重新确认摆位、绳索、人员、急停、双机状态和现场安全后，才允许分别使用
以下双门槛；仅有环境变量或仅有参数均会拒绝：

```bash
V6R10_ACTIVE_SITE_CONFIRMED=YES \
  ./scripts/start_v11_robots.sh start active --site-safety-confirmed

V6R10_ACTIVE_SITE_CONFIRMED=YES \
  ./scripts/start_v6r10_workstation.sh start active --site-safety-confirmed
```

active 应从低幅单球开始，再做左右交替；R2 由现场人员按既有 Controller 流程操作。
加载、shadow、transport ACK 均不能表述为站稳、击球或回球通过。

## 回滚

仅停止本部署记录的 workstation owner 和原 `v11_dual_robots` 会话，按 PID/starttime
复核退出，并确认双 command topic 无 publisher。旧 V3+V9 文件和 ROS 11312 从未
覆盖；需要恢复时按 `yichao_v3_v9/DEPLOY_V9.md` 的两个旧入口重新启动，不做文件回滚。
