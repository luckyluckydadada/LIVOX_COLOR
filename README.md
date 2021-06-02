# LIVOX-COLOR-ROS
The livox color ros package is a tool for synchronizing livox lidar to generate color point cloud information. This node also subscribes to livox's /livox/lidar topic and camera's /hikrobot_camera/rgb topic to generate /livox/livox_color topic.
Detailed guidance reference:https://blog.csdn.net/weixin_41965898/article/details/117447600

# Install
```
mkdir -p ~/ws_livox_color/src
git clone https://github.com/luckyluckydadada/LIVOX_COLOR.git ~/ws_livox_color/src/livox_color
cd ~/ws_livox_color
catkin_make
```
# launch run
```
source ./devel/setup.bash 
roslaunch livox_color color_livox.launch
```
# launch run with rviz
```
source ./devel/setup.bash 
roslaunch livox_color color_livox_rviz.launch
```
