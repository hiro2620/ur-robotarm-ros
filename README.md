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

[INFO] [launch]: All log files can be found below /root/.ros/log/2025-04-08-09-18-25-861240-HP-EliteBook-630-1006
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dashboard_client-2]: process started with pid [1010]
[INFO] [controller_stopper_node-4]: process started with pid [1014]
[INFO] [ur_ros2_control_node-1]: process started with pid [1008]
[INFO] [robot_state_helper-3]: process started with pid [1012]
[INFO] [urscript_interface-5]: process started with pid [1016]
[INFO] [robot_state_publisher-6]: process started with pid [1018]
[INFO] [rviz2-7]: process started with pid [1020]
[INFO] [spawner-8]: process started with pid [1022]
[INFO] [spawner-9]: process started with pid [1024]
[ur_ros2_control_node-1] [WARN] [1744103906.269572800] [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
[ur_ros2_control_node-1] text not specified in the tf_prefix tag
[ur_ros2_control_node-1] [INFO] [1744103906.271443794] [resource_manager]: Loading hardware 'ur5' 
[ur_ros2_control_node-1] [INFO] [1744103906.282527352] [resource_manager]: Initialize hardware 'ur5' 
[ur_ros2_control_node-1] [INFO] [1744103906.283677085] [resource_manager]: Successful initialization of hardware 'ur5'
[ur_ros2_control_node-1] [INFO] [1744103906.284310945] [resource_manager]: 'configure' hardware 'ur5' 
[ur_ros2_control_node-1] [INFO] [1744103906.284349983] [URPositionHardwareInterface]: Starting ...please wait...
[ur_ros2_control_node-1] [INFO] [1744103906.284390800] [URPositionHardwareInterface]: Initializing driver...
[rviz2-7] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[controller_stopper_node-4] [INFO] [1744103906.300177304] [Controller stopper]: Waiting for switch controller service to come up on controller_manager/switch_controller
[robot_state_publisher-6] [INFO] [1744103906.306373653] [robot_state_publisher]: got segment base
[robot_state_publisher-6] [INFO] [1744103906.306914444] [robot_state_publisher]: got segment base_link
[robot_state_publisher-6] [INFO] [1744103906.306962157] [robot_state_publisher]: got segment base_link_inertia
[robot_state_publisher-6] [INFO] [1744103906.306990008] [robot_state_publisher]: got segment flange
[robot_state_publisher-6] [INFO] [1744103906.307015340] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-6] [INFO] [1744103906.307038749] [robot_state_publisher]: got segment ft_frame
[robot_state_publisher-6] [INFO] [1744103906.307061646] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-6] [INFO] [1744103906.307084855] [robot_state_publisher]: got segment tool0
[robot_state_publisher-6] [INFO] [1744103906.307121598] [robot_state_publisher]: got segment upper_arm_link
[robot_state_publisher-6] [INFO] [1744103906.307147413] [robot_state_publisher]: got segment world
[robot_state_publisher-6] [INFO] [1744103906.307169911] [robot_state_publisher]: got segment wrist_1_link
[robot_state_publisher-6] [INFO] [1744103906.307192900] [robot_state_publisher]: got segment wrist_2_link
[robot_state_publisher-6] [INFO] [1744103906.307214816] [robot_state_publisher]: got segment wrist_3_link
[ur_ros2_control_node-1] [INFO] [1744103906.398320595] [UR_Client_Library:]: SCHED_FIFO OK, priority 99
[dashboard_client-2] [INFO] [1744103906.494347469] [UR_Client_Library:]: Connected: Universal Robots Dashboard Server
[dashboard_client-2] 
[ur_ros2_control_node-1] [INFO] [1744103906.499304834] [UR_Client_Library:]: Negotiated RTDE protocol version to 2.
[ur_ros2_control_node-1] [INFO] [1744103906.505549782] [UR_Client_Library:]: Setting up RTDE communication with frequency 125.000000
[rviz2-7] [INFO] [1744103906.548048123] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1744103906.548308793] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-7] [INFO] [1744103906.587010061] [rviz2]: Stereo is NOT SUPPORTED
[spawner-9] [INFO] [1744103906.719828313] [spawner_joint_trajectory_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-8] [INFO] [1744103906.727084599] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[ur_ros2_control_node-1] [ERROR] [1744103907.091038395] [UR_Client_Library:]: The following variables are not recognized by the robot:
[ur_ros2_control_node-1]   - 'tcp_offset'
[ur_ros2_control_node-1] Either your output recipe contains errors or the urcontrol version does not support them.
[ur_ros2_control_node-1] [INFO] [1744103907.091208828] [UR_Client_Library:]: Stopping primary client pipeline
[ur_ros2_control_node-1] [FATAL] [1744103908.143667934] [URPositionHardwareInterface]: The following variables are not recognized by the robot:
[ur_ros2_control_node-1]   - 'tcp_offset'
[ur_ros2_control_node-1] Either your output recipe contains errors or the urcontrol version does not support them.
[ur_ros2_control_node-1] [INFO] [1744103908.143770768] [resource_manager]: Failed to 'configure' hardware 'ur5'
[ur_ros2_control_node-1] terminate called after throwing an instance of 'std::runtime_error'
[ur_ros2_control_node-1]   what():  Failed to set the initial state of the component : ur5 to active
[ur_ros2_control_node-1] Stack trace (most recent call last):
[ur_ros2_control_node-1] #13   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in 
[ur_ros2_control_node-1] #12   Source "/usr/include/c++/11/bits/shared_ptr_base.h", line 704, in _start [0x56d0a3a0dca4]
[ur_ros2_control_node-1]         702:       ~__shared_count() noexcept
[ur_ros2_control_node-1]         703:       {
[ur_ros2_control_node-1]       > 704:   if (_M_pi != nullptr)
[ur_ros2_control_node-1]         705:     _M_pi->_M_release();
[ur_ros2_control_node-1]         706:       }
[ur_ros2_control_node-1] #11   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x722679e1be3f, in __libc_start_main
[ur_ros2_control_node-1] #10   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x722679e1bd8f, in 
[ur_ros2_control_node-1] #9  | Source "/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/src/ur_ros2_control_node.cpp", line 56, in make_shared<controller_manager::ControllerManager, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |    54:   std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
[ur_ros2_control_node-1]     |    55:   // create controller manager instance
[ur_ros2_control_node-1]     | >  56:   auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");
[ur_ros2_control_node-1]     |    57: 
[ur_ros2_control_node-1]     |    58:   // control loop thread
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr.h", line 879, in allocate_shared<controller_manager::ControllerManager, std::allocator<controller_manager::ControllerManager>, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   877:       typedef typename std::remove_cv<_Tp>::type _Tp_nc;
[ur_ros2_control_node-1]     |   878:       return std::allocate_shared<_Tp>(std::allocator<_Tp_nc>(),
[ur_ros2_control_node-1]     | > 879:                                  std::forward<_Args>(__args)...);
[ur_ros2_control_node-1]     |   880:     }
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr.h", line 863, in shared_ptr<std::allocator<controller_manager::ControllerManager>, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   862:       return shared_ptr<_Tp>(_Sp_alloc_shared_tag<_Alloc>{__a},
[ur_ros2_control_node-1]     | > 863:                        std::forward<_Args>(__args)...);
[ur_ros2_control_node-1]     |   864:     }
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr.h", line 409, in __shared_ptr<std::allocator<controller_manager::ControllerManager>, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   407:       template<typename _Alloc, typename... _Args>
[ur_ros2_control_node-1]     |   408:   shared_ptr(_Sp_alloc_shared_tag<_Alloc> __tag, _Args&&... __args)
[ur_ros2_control_node-1]     | > 409:   : __shared_ptr<_Tp>(__tag, std::forward<_Args>(__args)...)
[ur_ros2_control_node-1]     |   410:   { }
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr_base.h", line 1342, in __shared_count<controller_manager::ControllerManager, std::allocator<controller_manager::ControllerManager>, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |  1340:       template<typename _Alloc, typename... _Args>
[ur_ros2_control_node-1]     |  1341:   __shared_ptr(_Sp_alloc_shared_tag<_Alloc> __tag, _Args&&... __args)
[ur_ros2_control_node-1]     | >1342:   : _M_ptr(), _M_refcount(_M_ptr, __tag, std::forward<_Args>(__args)...)
[ur_ros2_control_node-1]     |  1343:   { _M_enable_shared_from_this_with(_M_ptr); }
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr_base.h", line 650, in _Sp_counted_ptr_inplace<std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   648:     auto __guard = std::__allocate_guarded(__a2);
[ur_ros2_control_node-1]     |   649:     _Sp_cp_type* __mem = __guard.get();
[ur_ros2_control_node-1]     | > 650:     auto __pi = ::new (__mem)
[ur_ros2_control_node-1]     |   651:       _Sp_cp_type(__a._M_a, std::forward<_Args>(__args)...);
[ur_ros2_control_node-1]     |   652:     __guard = nullptr;
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/shared_ptr_base.h", line 519, in construct<controller_manager::ControllerManager, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   517:     // _GLIBCXX_RESOLVE_LIB_DEFECTS
[ur_ros2_control_node-1]     |   518:     // 2070.  allocate_shared should use allocator_traits<A>::construct
[ur_ros2_control_node-1]     | > 519:     allocator_traits<_Alloc>::construct(__a, _M_ptr(),
[ur_ros2_control_node-1]     |   520:         std::forward<_Args>(__args)...); // might throw
[ur_ros2_control_node-1]     |   521:   }
[ur_ros2_control_node-1]     | Source "/usr/include/c++/11/bits/alloc_traits.h", line 516, in construct<controller_manager::ControllerManager, std::shared_ptr<rclcpp::Executor>&, char const (&)[19]>
[ur_ros2_control_node-1]     |   514:   {
[ur_ros2_control_node-1]     |   515: #if __cplusplus <= 201703L
[ur_ros2_control_node-1]     | > 516:     __a.construct(__p, std::forward<_Args>(__args)...);
[ur_ros2_control_node-1]     |   517: #else
[ur_ros2_control_node-1]     |   518:     std::construct_at(__p, std::forward<_Args>(__args)...);
[ur_ros2_control_node-1]       Source "/usr/include/c++/11/ext/new_allocator.h", line 162, in main [0x56d0a3a0d915]
[ur_ros2_control_node-1]         159:   void
[ur_ros2_control_node-1]         160:   construct(_Up* __p, _Args&&... __args)
[ur_ros2_control_node-1]         161:   noexcept(std::is_nothrow_constructible<_Up, _Args...>::value)
[ur_ros2_control_node-1]       > 162:   { ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }
[ur_ros2_control_node-1]         163: 
[ur_ros2_control_node-1]         164:       template<typename _Up>
[ur_ros2_control_node-1]         165:   void
[ur_ros2_control_node-1] #8    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0x72267a52674a, in controller_manager::ControllerManager::ControllerManager(std::shared_ptr<rclcpp::Executor>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, rclcpp::NodeOptions const&)
[ur_ros2_control_node-1] #7    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0x72267a50c34c, in 
[ur_ros2_control_node-1] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x72267a0e94d7, in __cxa_throw
[ur_ros2_control_node-1] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x72267a0e9276, in std::terminate()
[ur_ros2_control_node-1] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x72267a0e920b, in 
[ur_ros2_control_node-1] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x72267a0ddb9d, in 
[ur_ros2_control_node-1] #2    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x722679e1a7f2, in abort
[ur_ros2_control_node-1] #1    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x722679e34475, in raise
[ur_ros2_control_node-1] #0    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x722679e889fc, in pthread_kill
[ur_ros2_control_node-1] Aborted (Signal sent by tkill() 1008 0)
[ERROR] [ur_ros2_control_node-1]: process has died [pid 1008, exit code -6, cmd '/ros2_ws/install/ur_robot_driver/lib/ur_robot_driver/ur_ros2_control_node --ros-args --params-file /tmp/launch_params_r9cqsqdv --params-file /ros2_ws/install/ur_robot_driver/share/ur_robot_driver/config/ur5_update_rate.yaml --params-file /tmp/launch_params_uh4xnfwy'].
[spawner-9] [FATAL] [1744103916.741580343] [spawner_joint_trajectory_controller]: Could not contact service /controller_manager/list_controllers
[spawner-8] [FATAL] [1744103916.751101357] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[ERROR] [spawner-9]: process has died [pid 1024, exit code 1, cmd '/opt/ros/humble/lib/controller_manager/spawner --controller-manager /controller_manager --controller-manager-timeout 10 --inactive joint_trajectory_controller forward_velocity_controller forward_position_controller force_mode_controller passthrough_trajectory_controller freedrive_mode_controller --ros-args'].
[ERROR] [spawner-8]: process has died [pid 1022, exit code 1, cmd '/opt/ros/humble/lib/controller_manager/spawner --controller-manager /controller_manager --controller-manager-timeout 10 joint_state_broadcaster io_and_status_controller speed_scaling_state_broadcaster force_torque_sensor_broadcaster tcp_pose_broadcaster ur_configuration_controller scaled_joint_trajectory_controller --ros-args'].