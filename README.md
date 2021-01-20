# 泊位系统

## 安装

1. 安装ros
2. 安装livox_ros_driver

## 运行

1. 命令行输入

   ```bash
   roslaunch lidar_system start.launch
   ```

   并回车

## 数据流结构

1. livox_ros_driver  --->  reciver
2. reciver ----------------> transform
3. transform -------------> cluster
4. cluster ----------------->image_ui
5. state_controler ------> image_ui
6. image_ui --------------> state_controler