# Yichao Monitor

复用 Fixed Monitor 的 state/torso/ball/command 展示，上层状态改订阅
`/doubles/yichao/status`。模块单独运行默认 HTTP 8089；新的双入口工作站脚本指定 **8090**，与旧 Yichao 8089、原 Fixed 8088 分开。

使用页面开始录制、停止保存；从列表加载后可播放/暂停、倍速和拖动。
回放只改变浏览器显示，不发布机器人或运行参数。点击“实时”恢复实时订阅显示。
录制存入 `output/monitor_recordings/`，支持 JSONL 下载和导入，关闭浏览器不停止录制。
旧录制缺少的模型/阶段字段保持缺失，不生成不存在的 ACK 或有效球。

启动 `PYTHONPATH=src .venv/bin/python -m yichao_v3_v9.monitor.server`，
离线 UI 检查加 `--demo --host 127.0.0.1 --port 18089`。默认不创建 ROS 参数发布者；
需要原 Fixed 的参数调整功能时显式使用 `--enable-config`。
