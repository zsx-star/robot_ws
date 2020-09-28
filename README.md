# 三轮机器人

## 键盘控制节点
sudo chmod a+r /dev/input/by-path/*
roslaunch keyboard_control keyboard.launch

## 机器人底层控制节点
roslaunch triwheel_robot_driver robot_driver.launch

## 键盘控制机器人节点
roslaunch triwheel_robot_driver robot_keyboard.launch

## 导航
rosservice call /global_localization "{}" 
roslaunch robot_nav ctrl_wan.launch


## 串口永久获得权限
sudo gedit /etc/udev/rules.d/70-ttyUSB.rules
添加一行
KERNEL=="ttyUSB*", OWNER="root", GROUP="root", MODE="0666" 


## dependencies
- tinyxml2 xml解析器

```bash
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```

# autolabor差动机器人
## cartographer建图
roslaunch slam autolabor_cartographer_slam_for_one_lidar.launch  #开始单雷达建图
cd robot_ws/src/slam/map/cartographer && ./create_map_stop       #停止建图并保存

## cartographer定位+move_based导航+teb_local_planner规划器
roslaunch robot_nav nav_autolabor_cartographer_for_one_lidar.launch  #开始单雷达导航

使用键盘控制机器人移动，直到定位成功，然后选择目标点开始导航， 定位成功后按数字9键禁用键盘控制，以保证车辆由导航模块控制，可按0键重新键盘接管控制




