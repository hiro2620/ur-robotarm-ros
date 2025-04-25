## 環境構築
### URCap
#### 1. externalContorolをインストール
PolyscopeにexternalControlというURCapをインストールする。
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver リポジトリの`/ur_robot_driver/resources/externalcontrol-x.x.x.urcap`をUSBメモリに入れて読み込む。
#### 2. プログラム作成
ティーチペンダント(UR5に付いているタブレット)で、Setup Robot > URCapsで、externalControlブロックを入れたプログラムを作成し、適当な名前で保存する。
externalControlの設定でPCのIPアドレスを設定する。(ポートはデフォルトの50002のままで良い。ドライバのlaunchオプションで設定できる。)
### PC
#### 1. Dockerコンテナをビルドする。
```bash
docker compose build
```
#### 2. コンテナ内のターミナルを開く。
コマンドは基本的にすべてコンテナ内ターミナルで実行する。
```bash
./run.sh
```
## 実行方法
### シミュレータでロボットを動かしてみる
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5
```

### 実機を動かしてみる
#### 1. ドライバを起動
ur5のコントローラとトルクフォースセンサ(ft300)のドライバが以下のコマンドで起動する。
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=${ROBOT_IP} use_sim_time:=false launch_rviz:=false initial_joint_controller:=forward_position_controller
```
MoveIt2のGUI(Rviz)でプランニングするときは、`initial_joint_controller:=forward_position_controller`を外す必要がある。(プランニングの実行は`scaled_trajectory_controller`で行われるように設定されている。)

#### 2. ロボット側のプログラム実行
環境構築でロボット側で作成したプログラムをティーチペンダントから起動する。
ros2のドライバ起動ターミナルに`reverse control connected`のようなメッセージが出れば正常に起動されている。

#### 3. MoveIt2を起動
逆運動学計算をする`moveit_servo`が起動するので、自作プログラムで操作する場合も起動する。
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

#### 4. プログラム実行
```bash
cd /src/my_packages/ur5_ft_control/scripts
python3 xxx.py
```

### メモ
- `moveit_servo`の有効化
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

- TFセンサの値を取得
```bash
ros2 topic echo /force_torque_sensor_broadcaster/wrench
```

- urdfとsrdfでur5とtfセンサを繋げたモデルを作るのは現状うまくいっていない。(使っていない。)