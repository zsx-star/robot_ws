
# gmapping demo
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch     #gazabo (environment_model/robot_model/sensor_model  )
roslaunch mbot_navigation gmapping_demo.launch         #gmapping

roslaunch mbot_teleop mbot_teleop.launch               #tele_control
rosrun map_server map_saver -f map                     #save_map

#navigation demo
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch
roslaunch mbot_navigation nav_cloister_demo.launch

#auto slam (exploring_slam)
roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch
roslaunch mbot_navigation exploring_slam_domo.launch
rosrun mbot_navigation exploring_slam.py

#amcl resample all partials
rosservice call /global_localization "{}"


