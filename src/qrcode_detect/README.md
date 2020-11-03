# 摄像头二维码检测辅助定位

## 1. 摄像头标定
roslaunch camera_driver raw_display.launch 
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.61 image:=/image_raw

## 2. 二维码生成
rosrun aruco optimalmarkers  num prefix size
* **`num`** 生成二维码的个数
* **`prefix`** 生成二维码的文件名前缀
* **`size`** 生成二维码的像素尺寸
rosrun aruco optimalmarkers  5 ID_ 500

## 3. 二维码检测与定位
roslaunch qrcode_detect detect.launch

/aruco_single/pose    [geometry_msgs/PoseStamped]
/aruco_single/marker  [visualization_msgs/Marker]
/aruco_single/result  [sensor_msgs/Image]
