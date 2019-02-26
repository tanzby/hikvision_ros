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
roslaunch hikvision_ros hikvision_ros
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
roslaunch hikvision_ros hikvision_ros ip_addr:=192.168.5.100 password:=123456
```



