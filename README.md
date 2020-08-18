<img src="./docs/images/IWT.png" align="right"
     title="IWT logo" width="184" height="55">

# nimbus_pnp_demo

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![PCL: version-1.8](https://img.shields.io/badge/PCL-version%3A%201.8-yellowgreen)](https://img.shields.io/badge/PCL-version%3A%201.8-yellowgreen)


|Branch    | ROS Distro | Status (travis)  | Github Action |
|----------|------------|-----------|----------------------|
|master    | [![ROS: Melodic](https://img.shields.io/badge/ROS-Melodic-blue)](https://img.shields.io/badge/ROS-Melodic-blue)    |[![Build Status](https://travis-ci.org/prachandabhanu/nimbus_pnp_demo.svg?branch=master)](https://travis-ci.org/prachandabhanu/nimbus_pnp_demo)| ![Nimbus](https://github.com/prachandabhanu/nimbus_pnp_demo/workflows/Nimbus/badge.svg?branch=master) |

## Setting up the evnironment

### Nimbus Evnvironment 

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LernFabrik/nimbus_pnp_demo.git
cd ..
source /opt/ros/melodic/setup.bash
rosdep install --ignore-src --from-paths src -y
catkin init
catkin build
source  ~/catkin_ws/devel/setup.bash
```

### KUKA IIWA7 Environment
```
mkdir -p ~/catkin_kuka_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LernFabrik/kuka_iiwa7_ros.git
cd ..
source /opt/ros/melodic/setup.bash
rosdep install --ignore-src --from-paths src -y
catkin init
catkin build
source  ~/catkin_kuka_ws/devel/setup.bash
```

## Run Box detection

`roslaunch box_detector box_detector.launch`


Read Documentation
1. [Cloud](https://github.com/prachandabhanu/nimbus_pnp_demo/blob/master/doc/rosdoc/detector/doc/html/index.html)
