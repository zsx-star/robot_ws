# 生成机器人导航目标

1. 利用map_server载入地图文件
2. 利用rviz显示地图,并使用2D Nav Goal工具标记目标点,目标点按照顺序逐个标记,并手工距离对应的标签
3. generate_goal_node 订阅/move_base_simple/goal,并将目标点保存在xml文件中以备使用
4. 目标文件生成完毕后,手动修改目标点对应的标签


## dependencies
- tinyxml2 xml解析器
安装
```bash
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```
头文件
```c++
#include<tinyxml2.h>
```
链接库
```
target_link_libraries(node_name /usr/local/lib/libtinyxml2.a)
```


也可使用opencv载入地图图片，然后使用图片点击回调保存目标点