# Robotaxi planning & control

## 一、介绍&配置

本项目帮助开发人员快速建立一个用于园区展示、科研教学的自动驾驶系统，具体配置如下：
* 线控底盘
* GPS/IMU
* x86 or arm CPU (4核以上)
* 8GB 内存以上
* Ubuntu 18.04
* ROS melodic

项目在两个最大的开源自动驾驶项目Apollo和Autoware的基础上进行了整理代码、修复bug，项目经过了仿真、实车测试验证，目前主要功能模块有：
* canbus
* drivers
* localization
* control
* planning

项目提供了轻量级的自动驾驶规划与控制程序，开发人员可以在此基础上做优化，添加高精度地图、感知、预测模块等操作

## 二、安装

1. 安装catkin build编译工具
   
```bash
sudo apt-get install python-catkin-tools
```

2. 安装jsk相关
```bash
sudo apt-get install ros-melodic-jsk-common ros-melodic-jsk-recognition-msgs ros-melodic-jsk-common-msgs ros-melodic-jsk-rviz-plugins
```

3. 安装llh2utm依赖
```bash
sudo apt-get install libgeographic-dev
```

4. 安装串口读取的ros-serial 库
```bash
sudo apt-get install ros-melodic-serial
```

## 三、操作说明
### 1. 构建 workspace
```bash
mkdir -p pnc/src
cd pnc/src/
```
推荐：克隆此项目源码canbus、common、control...全部移动到pnc/src路径下(后面有.表示clone到当前目录下)
```bash
(从github获取)git clone git@github.com:snail333/planning-control.git .
(或从gitee获取)git clone git@gitee.com:zhangchunxinha/planning-control.git .
```
__注意__： 使用ssh克隆代码库需要设置SSH公钥。

或使用下面的命令克隆代码库：
```bash
(从github获取)git clone https://github.com/snail333/planning-control.git .
(或从gitee获取)git clone https://gitee.com/zhangchunxinha/planning-control.git .
```

编译
```bash
cd ..
catkin build
```

### 2. 运行系统

在home路径下打开.bashrc文件，在最后一行添加：
```bash
source ～/pnc/devel/setup.bash
```

2.1 开始自动驾驶：
```bash
cd src/scripts
./run.sh
```

2.2 记录数据rosbag：
```bash
cd src/scripts
./record.sh
```

2.3 仿真程序：
```bash
cd src/scripts
roslaunch simulation.launch
```
说明：仿真需要在可视化界面Rviz上点击“2D Pose Estimate",再点击黑色背景区域，添加车辆初始位置