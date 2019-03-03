# hikvision_ros
simple preview bridge node of  hikvision IP camera for ROS 



## dependance

* cv_bridge
* image_transport
* OpenCV3

## usage



***instant run***

```sh
git clone https://github.com/tanzby/hikvision_ros.git
cd hikvision_ros
mkdir build && cd build
cmake ..
make
```

Publish ` sensor_msgs::Image`

```sh
roscore
source <path/to/devel/setup.sh>
roslaunch hikvision_ros hik.launch
```



***parameters***

You can specify some camera and steam parameters by `hik.launch`

```xml
<arg name="ip_addr" default="192.168.5.100"/>
<arg name="user_name" default="admin"/>
<arg name="password" default="admin"/>
<arg name="port" default="8000"/>
<arg name="channel" default="1"/>
```

Or in command line

```sh
roslaunch hikvision_ros hik.launch ip_addr:=192.168.5.100 password:=123456
```



***support for camera_calibration***

you can use [camera_calibration](http://wiki.ros.org/camera_calibration/)  to calibrate hikvision camera, **hik_ros**  provides *set_camera_info* sevice for receiving and storing camera's calibration parameters. 

***example***

```sh
# open camera
roslaunch hikvision_ros hik.launch

# see topic name
rostopic list

# [output]
# ➜  ~ rostopic list
# /hik_cam_node/camera_info
# /hik_cam_node/hik_camera
# ...

# calibrate
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/hik_cam_node/hik_camera  camera:=/hik_cam_node/hik_camera
```

then begin calibration. After calibration you straightly press **commit** button,  **hik_ros** has the ability to save the calibration parameter to `camera_info_url`, which is set in launch file OR use default path (  `~/.ros/camera_info` )   

```sh
# [output]
#[ INFO] [1551582484.454024618]: New camera info received
#[ INFO] [1551582484.454296067]: [ hik_camera ] Write camera_info to ~/.ros/camera_info/hik_camera.yaml success.
```

