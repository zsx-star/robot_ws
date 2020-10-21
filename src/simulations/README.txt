
启动gmapping演示
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch     #gazabo仿真环境(环境模型、机器人模型、传感器模型)
roslaunch mbot_navigation gmapping_demo.launch         #gmapping
roslaunch mbot_teleop mbot_teleop.launch
rosrun map_server map_saver -f map  保存地图

导航仿真 
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch
roslaunch mbot_navigation nav_cloister_demo.launch


导航slam仿真
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch
roslaunch mbot_navigation exploring_slam_demo.launch

自主探索slam仿真
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch
roslaunch mobot_navigation exploring_slam_domo.launch
rosrun mbot_navigation exploring_slam.py

rosservice call /global_localization "{}"


