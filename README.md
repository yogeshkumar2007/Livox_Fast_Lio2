1. create a workspace
2. install livox driver2 and sdk
3. check the ip addr 
4. Terminal 1 - ros2 launch livox_ros_driver2 msg_MID360_launch.py
5. Terminal 2 - ros2 run imu_unit_converter imu_converter
6. Terminal 3 - ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml^C
7. Terminal 4(if dynamic map required) - ros2 run dynamic_map_filter dynamic_filter
8. in RViz ADD the PointCloude2 and set the topic to Dynamic
