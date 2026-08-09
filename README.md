create a workspace
install livox driver2 and sdk
check the ip addr 
Terminal 1 - ros2 launch livox_ros_driver2 msg_MID360_launch.py
Terminal 2 - ros2 run imu_unit_converter imu_converter
Terminal 3 - ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml^C
Terminal 4(if dynamic map required) - ros2 run dynamic_map_filter dynamic_filter
in RViz ADD the PointCloude2 and set the topic to Dynamic
