## 環境構築
### URCap
1. PolyscopeにexternalControlというURCapをインストールする。

Setup Robot > URCapsで+

## 実行方法
### シミュレータ + Move It
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5
```

### 実機 + Move It
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=${ROBOT_IP} use_sim_time:=false launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

### トルクセンサあり
```bash
ros2 launch ur5_ft_control ur5_ft_control.launch.py
```