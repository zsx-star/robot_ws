# robot_ws
The robot platform in h-u-a-m-a-n

1. 修改键盘权限
sudo chmod a+r /dev/input/by-path/*
#### 键盘控制节点
roslaunch keyboard_control keyboard.launch

#### 机器人底层控制节点
roslaunch triwheel_robot_driver robot_driver.launch

#### 键盘控制机器人节点
roslaunch triwheel_robot_driver robot_keyboard.launch


#### 建图





串口永久获得权限
sudo gedit /etc/udev/rules.d/70-ttyUSB.rules

KERNEL=="ttyUSB*", OWNER="root", GROUP="root", MODE="0666" 
