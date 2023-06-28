# smarty_arm_interface

This repository contains the ROS interface code for smarty arm.

## Dependencies
```
git clone https://github.com/Chunpeng19/smarty_arm_control
```

## Usage

Left smarty arm:
```
sudo [workspace]/src/smarty_arm_control/build/smarty_arm_control l
```
```
roslaunch smarty_arm_interface smarty_arm_interface.launch smarty_arm_type:=l
```

Right smarty arm:
```
sudo [workspace]/src/smarty_arm_control/build/smarty_arm_control r
```
```
roslaunch smarty_arm_interface smarty_arm_interface.launch smarty_arm_type:=r
```
