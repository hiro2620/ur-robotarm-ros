from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パラメータの設定
    force_threshold_arg = DeclareLaunchArgument(
        'force_threshold', 
        default_value='10.0',
        description='力の閾値 (N) - この値を超えるとロボットが動作を開始'
    )
    
    torque_threshold_arg = DeclareLaunchArgument(
        'torque_threshold', 
        default_value='1.0',
        description='トルクの閾値 (Nm) - この値を超えるとロボットが動作を開始'
    )
    
    position_scaling_arg = DeclareLaunchArgument(
        'position_scaling', 
        default_value='0.001',
        description='位置スケーリング係数 (m/N) - 力から位置変化へのスケーリング係数'
    )
    
    rotation_scaling_arg = DeclareLaunchArgument(
        'rotation_scaling', 
        default_value='0.01',
        description='回転スケーリング係数 (rad/Nm) - トルクから回転変化へのスケーリング係数'
    )
    
    planning_group_arg = DeclareLaunchArgument(
        'planning_group', 
        default_value='ur_manipulator',
        description='MoveIt2のプランニンググループ名'
    )
    
    # MoveIt2の起動ファイルを含める（UR5用）
    # 注意: パッケージがインストールされていることを前提としています
    moveit_config_package = FindPackageShare('ur_moveit_config')
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_package, 'launch', 'ur_moveit.launch.py'])
        ]),
        # MoveIt2の起動パラメータが必要な場合は追加
        launch_arguments={
            'ur_type': 'ur5',
            'use_fake_hardware': 'false',
            'launch_rviz': 'true'
        }.items()
    )
    
    # UR5 FTコントローラノードの設定
    ur5_ft_controller_node = Node(
        package='ur5_ft_control',
        executable='ur5_ft_controller',
        name='ur5_ft_controller',
        output='screen',
        parameters=[{
            'force_threshold': LaunchConfiguration('force_threshold'),
            'torque_threshold': LaunchConfiguration('torque_threshold'),
            'position_scaling': LaunchConfiguration('position_scaling'),
            'rotation_scaling': LaunchConfiguration('rotation_scaling'),
            'planning_group': LaunchConfiguration('planning_group'),
        }],
    )
    
    # LaunchDescriptionの作成
    return LaunchDescription([
        # パラメータ宣言
        force_threshold_arg,
        torque_threshold_arg,
        position_scaling_arg,
        rotation_scaling_arg,
        planning_group_arg,
        
        # MoveIt2の起動
        moveit_launch,
        
        # UR5 FT制御ノード
        ur5_ft_controller_node,
    ])