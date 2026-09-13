# PingPong infrastructure

当前仓库同时保留两套独立的 Yichao Planner 适配；原 Fixed、Predictor、Controller 保持独立。

- V6R10 actor-only + V11 i42500：见 [实现与双入口说明](yichao_v6r10_v11/README.md)、[部署说明](yichao_v6r10_v11/DEPLOY.md) 和 [交付记录](doc/V6R10_ACTOR_ONLY_V11_DEPLOYED_2026-09-13.md)。
- V3 + V9：见 [部署说明](yichao_v3_v9/README.md) 和 [Monitor](yichao_v3_v9/MONITOR.md)。

V6R10 路径复用 Fixed relay 状态机，以 model630 联合双机 actor 产生 lateral-Y proposal，生产路径关闭 learned barrier，并通过确定性 0.50 m hard guard 和 V11 双侧命令 ACK 后释放 HIT。机器人端 V11 Controller 源码未在本次适配中修改；仓库保存接口合同、启动器和资产哈希。

2026-09-10 最新现场反馈：新标定生效后，双机站稳，来球时移动换位并尝试接球。
尚无成功回球率或长期稳定性统计。

此仓库提供源码，不携带 ONNX/checkpoint、设备环境、现场日志和重写前归档；模型 sidecar、SHA256 和部署 manifest 用于还原及核对外部资产。
其它 doc 下旧说明是历史记录，启动以新版 README 为准。
