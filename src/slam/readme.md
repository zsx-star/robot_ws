### 机器人端执行
1. gmapping, driver, lidar, tf(lidar->base_link)
roslaunch slam robot_gmapping.launch

### 电脑端执行
1. rviz, keyboard_control
roslaunch slam gmapping_remote.launch

2. laser_scan_matcher (基于点云匹配的里程计, 可选)
roslaunch slam laser_scan_matcher.launch 

rosrun map_server map_saver -f map_name
