# iahrs_ros

## Overview

This is a driver package for the ROBOR's RB-SDA-v1 from http://robor.co.kr/?page_id=1101 .

### Original Source

The original source (not support ROS) is referenced below:

https://github.com/page365/iahrs_linux

## Installation

```sh
cd ~/catkin_ws/src
git clone https://github.com/lim271/iahrs_ros.git
cd ~/catkin_ws && catkin_make
```

# Run

Run the driver like so:

```sh
roslaunch iahrs_ros iahrs_ros.launch
```

