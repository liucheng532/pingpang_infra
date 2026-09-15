# 项目说明
这个仓库包含两个主要部分：
- `vision/mocapapi-demo-py/`：使用动捕系统获取乒乓球位置，并通过ROS1发布预测结果。
- `g1_gym_deploy/` 与 `unitree_sdk2/`：用于部署和运行Unitree G1机器人的策略以及底层SDK。

以下内容帮助你快速运行最常用的两个功能。

## Vision：动捕到ROS1 (TableTennis)
1. 打开终端并启动ROS核心：
   ```
   roscore
   ```
2. 新建终端并进入动捕示例目录：
   ```
   cd /home/unitree/rjl/sports-mimic-deploy/vision/mocapapi-demo-py
   python TableTennis.py
   ```
   - 程序会自动连接Hybrid Data Server（默认UDP 7012）。
   - 当球穿越桌面中心平面时，会预测未来0~0.3秒内的落点。
3. ROS1发布的Topic：
   - `/predicted_ball_position` (`geometry_msgs/PointStamped`)
   - `/predicted_ball_velocity` (`geometry_msgs/TwistStamped`)
   - `/predicted_ball_predict_time` (`std_msgs/Float32`)

若只需要把动捕刚体发布到ROS1，可运行：
```
cd /home/unitree/rjl/sports-mimic-deploy/vision/mocapapi-demo-py
python mocap_to_ros.py
```

## G1策略部署（速查）
1. 启动机器人控制程序：
   ```
   ./g1_control eth0
   ```
2. 在另一个终端下发策略：
   ```
   cd /home/unitree/rjl/sports-mimic-deploy/g1_gym_deploy/scripts
   python deploy_policy.py
   ```

## Wi-Fi调试速记
```
sudo usb_modeswitch -K -v 0bda -p 1a2b
sudo modprobe 8851bu
iwlist wlan0 channel
sudo nmcli connection modify "RobotOnly" 802-11-wireless.band a
nmcli connection show
nmcli connection up "RobotOnly"
```

## 桌面/远程显示
```
sudo systemctl stop gdm3
sudo /etc/NX/nxserver --restart
```

## connect yinghui computer in ros
export ROS_MASTER_URI=http://192.168.3.11:11311 ## host ip
export ROS_IP=192.168.3.100 ## my ip

## 变更记录
- 2025-11-27：`TableTennis.py` 改为ROS1发布，无需ROS2环境。