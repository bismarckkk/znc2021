# 部署说明  

## 项目依赖  

项目在`ubuntu18.04`、`ros-melodic`、`python2`环境下通过测试 

### apt包  

* ros-melodic-navigation  
* ros-melodic-amcl  
* ros-melodic-teb-local-planner  

> apt依赖项可能不全，有几个不太记得住了  

### python库  

* rospy（应与ros一同安装）  
* numpy  

> 如果发现在第一个拐弯处撞墙请排查`vel_controller`依赖是否齐全，能否正常启动

## 目录说明  

此程序有部分代码强依赖与目录结构，改变可能无法运行  
```text
src
├─gazebo_pkg    // 官方提供模型包
├─start_game    // 官方提供计时包
└─race          // 主要功能实现
    ├─config        // teb、amcl相关参数文件
    ├─launch        // 一些启动文件
    │   ├─main.launch       // 启动除gazebo模型外的整个程序
    │   └─test.launch       // 启动除gazebo模型和vel_controller外的程序
    ├─maps          // 已经建好的地图
    ├─rviz          // 存储rviz样式
    └─scripts       // 自行编写的部分程序
        ├─move.py           // 用于调试的终点发布程序
        └─vel_controll.py   // 速度控制程序
```