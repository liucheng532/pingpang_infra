# 双打 Planner Web Monitor

双打页面复用单打 Monitor 的白色布局，展示一套共享 Predictor/Table/Racket，
以及 `table_right/66`、`table_left/198` 两套 robot state、planner command 和
runtime motion config。

## 启动

```bash
cd /home/odl/codebase/pingpang_doubles_v9_runtime
source /opt/ros/noetic/setup.bash

ROS_MASTER_URI=http://192.168.123.165:11311 \
ROS_IP=192.168.123.165 \
PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python -B -u \
-m ros_web_monitor.server --host 0.0.0.0 --port 8088
```

浏览器访问 `http://172.16.3.126:8088`。

## 数据与控制

页面订阅：

- `/doubles/table_right/state`、`/doubles/table_left/state`
- `/doubles/table_right/torso_pose_origin`、`/doubles/table_left/torso_pose_origin`
- `/doubles/table_right/command`、`/doubles/table_left/command`
- `/doubles/ball_prediction`
- `/doubles/v9_fixed_relay/status`、`/doubles/v9_cbf/status`

两路 torso 使用 `geometry_msgs/PoseStamped`，页面分别显示实际 XYZ、由 xyzw
四元数换算的 roll/pitch/yaw（度）以及接收年龄。其余默认双打 topic 使用
`std_msgs/String` JSON。

两张 robot 配置卡分别向 `POST /api/runtime_motion_config/{robot}` 提交
`goal_x/outward_y/home_y/outward_hold_s`。Monitor 将经过基础校验的配置发布到
`/doubles/runtime_motion_config`；Fixed Planner 负责组合几何校验、版本管理、
command 下发和双阶段 ack。配置只保存在运行内存中。

## Demo 与测试

```bash
python3 -u -m ros_web_monitor.server --demo --port 8088 --no-log
python3 -m pytest -q tests/test_ros_web_monitor.py
```

默认 JSONL 日志位于 `outputs/ros_web_monitor_logs/`，单文件 100 MB，保留 5 份。
HTTP 接口包括 `/api/snapshot`、`/api/events`、`/api/log/latest` 和 `/healthz`。
