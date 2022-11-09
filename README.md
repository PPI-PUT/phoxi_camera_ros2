# PHOXI_CAMERA_ROS2
ROS2 driver for PhoXi cameras with the concept of managed nodes. 
See [user manual](https://www.photoneo.com/downloads/phoxi-control) for more information.

https://user-images.githubusercontent.com/37396312/199533561-61334579-3e52-4fe0-b88e-f717877cce02.mp4


## Requirements
* PhoXi Control (tested with 1.9)
* ROS2 humble
* PCL (tested with 1.12)
* OpenCV (tested with 4.5.4)

Optional:
* Docker
* [Rocker](https://github.com/osrf/rocker)


## Installation & building
### Host
```
cd $HOME
mkdir -p phoxi_ws/src && cd phoxi_ws
git clone https://github.com/PPI-PUT/phoxi_camera_ros2 src/phoxi_camera_ros2
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to phoxi_camera_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On
```
or
### Docker
```
cd $HOME
mkdir -p phoxi_ws/src && cd phoxi_ws
git clone https://github.com/PPI-PUT/phoxi_camera_ros2 src/phoxi_camera_ros2
cd src/phoxi_camera_ros2/docker
```
Download and copy [PhoXi Control 1.9.1 (Ubuntu 20)](https://www.photoneo.com/downloads/phoxi-control) here as *PhotoneoPhoXiControlInstaller-1.9.1-Ubuntu20-STABLE.tar.gz*
```
./build.sh
```
After build finished, run container. If you used custom path for this package, you need to change volume path in run.sh file. 
```
./run.sh
cd phoxi_ws
colcon build --symlink-install --packages-up-to phoxi_camera_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On
```
To use another terminal, open new terminal window and use script.
```
./enter.sh
```

## Usage
1. Connect camera.
2. Configure param file (phoxi_camera_ros2/param/defaults.param.yaml).
3. Run main nodes - you don't need to run PhoXiControl by hand!
```
source install/setup.bash
ros2 launch phoxi_camera_ros2 phoxi_camera.launch.py 
```
In case of `software` mode, you may run it `with_rviz:=False` flag. 
Example service call:
```
ros2 service call /phoxi/phoxi_camera/depth_map phoxi_camera_msgs/srv/PhoxiImg "{save: true}"
```
TIFF format may require appropriate image viewer on your OS. All the services have the same request 
format. It is more convenient to write your own service client for your application purposes - 
responses contain camera output.


## Interface
Camera may operate only in a single trigger mode - `freerun` (topics) or `software` (services). 
Exposed interface varies depending on the mode selected.

### Parameters

| Name                      | Type   | Description                                             |
| ------------------------- | ------ | ------------------------------------------------------- |
| `trigger_mode`            | string | freerun (topics), software (services)                   |
| `operation_mode`          | string | camera, scanner, mode2d (only texture)                  |
| `hardware_id`             | string | Camera serial number                                    |
| `use_pointcloud`          | bool   | Use pointcloud output                                   |
| `use_color_camera_image`  | bool   | Use color image output (if exists)                      |
| `use_confidence_map`      | bool   | Use confidence map output                               |
| `use_depth_map`           | bool   | Use depth map output (meters)                           |
| `use_event_map`           | bool   | Use event map output                                    |
| `use_normal_map`          | bool   | Use normal map output                                   |
| `use_texture`             | bool   | Use texture output (gray or rgb)                        |
| `save_dir`                | string | Absolute path to data output for services               |
| `roi`                     | list   | Region of interest, 6 values (min x, y, z, max x, y, z) |
| `tf_prefix`               | string | TF prefix for frame link                                |
| `debug`                   | bool   | Print processing time                                   |

### Services

| Name                                     | Type                               | Description            |
| ---------------------------------------- | ---------------------------------- | ---------------------- |
| `/phoxi/phoxi_camera/cloud`              | phoxi_camera_msgs::srv::PhoxiCloud | cloud request          |
| `/phoxi/phoxi_camera/color_camera_image` | phoxi_camera_msgs::srv::PhoxiImg   | color image request    |
| `/phoxi/phoxi_camera/confidence_map`     | phoxi_camera_msgs::srv::PhoxiImg   | confidence map request |
| `/phoxi/phoxi_camera/depth_map`          | phoxi_camera_msgs::srv::PhoxiImg   | depth map request      |
| `/phoxi/phoxi_camera/event_map`          | phoxi_camera_msgs::srv::PhoxiImg   | event map request      |
| `/phoxi/phoxi_camera/full`               | phoxi_camera_msgs::srv::PhoxiFull  | all data request       |
| `/phoxi/phoxi_camera/normal_map`         | phoxi_camera_msgs::srv::PhoxiImg   | normal map request     |
| `/phoxi/phoxi_camera/texture`            | phoxi_camera_msgs::srv::PhoxiImg   | texture request        |


### Publications

| Name                                     | Type                          | Description    |
| ---------------------------------------- | ----------------------------- | -------------- |
| `/phoxi/phoxi_camera/cloud`              | sensor_msgs::msg::PointCloud2 | cloud          |
| `/phoxi/phoxi_camera/color_camera_image` | sensor_msgs::msg::Image       | color image    |
| `/phoxi/phoxi_camera/confidence_map`     | sensor_msgs::msg::Image       | confidence map |
| `/phoxi/phoxi_camera/depth_map`          | sensor_msgs::msg::Image       | depth map      |
| `/phoxi/phoxi_camera/event_map`          | sensor_msgs::msg::Image       | event map      |
| `/phoxi/phoxi_camera/normal_map`         | sensor_msgs::msg::Image       | normal map     |
| `/phoxi/phoxi_camera/texture`            | sensor_msgs::msg::Image       | texture        |


## Limitations
Software tested with MotionCam-3D (no color) on Ubuntu 20, but it should works with rest of cameras.
Some code still require review with respect to the time constraints. As I have some spare time 
(might not happen :)), all the camera API functionalities may be added periodically, same for cameras 
meshes.