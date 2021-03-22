# robot_ws
## 一、环境安装配置
### 1. 自动安装所有ros依赖包
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y

### 2. 三方依赖库
#### 2.1 tinyxml2 xml解析器
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install

### 3. 串口、键盘权限
添加串口永久权限 \
sudo gedit /etc/udev/rules.d/70-ttyUSB.rules
KERNEL=="ttyUSB*", OWNER="root", GROUP="root", MODE="0666" 

修改键盘权限
sudo chmod a+r /dev/input/by-path/*

端口映射
gedit /etc/udev/rules.d/com.rules
其中KERNELS的值通过 $ll /sys/class/tty/ttyUSB* 查看

ACTION=="add",KERNELS=="3-3.2:1.0",SUBSYSTEMS=="usb",MODE:="0666",SYMLINK+="lidar" 
ACTION=="add",KERNELS=="3-3.2:1.0",SUBSYSTEMS=="usb",MODE:="0666",SYMLINK+="robot_base" 


## 二、机器人功能
### 1. 键盘控制机器人
roslaunch triwheel_robot_driver robot_keyboard.launch

### 2. SLAM建图
roslaunch slam robot_gmapping.launch


## 导航
rosservice call /global_localization "{}" 
roslaunch robot_nav ctrl_wan.launch










# autolabor差动机器人
## cartographer建图
roslaunch slam autolabor_cartographer_slam_for_one_lidar.launch  #开始单雷达建图
cd robot_ws/src/slam/map/cartographer && ./create_map_stop       #停止建图并保存

## cartographer定位+move_based导航+teb_local_planner规划器
roslaunch robot_nav nav_autolabor_cartographer_for_one_lidar.launch  #开始单雷达导航

使用键盘控制机器人移动，直到定位成功，然后选择目标点开始导航， 定位成功后按数字9键禁用键盘控制，以保证车辆由导航模块控制，可按0键重新键盘接管控制




