2020年2月2日 廿一 初稿

## 一些简单的说明

本脚本基于 Python2 编写，仅使用 Pyhton 标准库，无需额外通过pip安装依赖

目录结构:

```bash
.
├── main.py
├── initialize.py
├── interactive.py
├── ros_module.py
├── resources
│   ├── gazebo_pkg
│   └── ucar_plane
├── pose.json
├── log.txt
└── vital_log.txt
├── README.MD
```

* `*.py`           脚本程序
* `resources`      资源文件夹，具体内容见下
* `pose.json`      目标点信息，包含一个表示姿态的四元数和三维坐标信息
* `log.txt`        脚本运行时显示在屏幕上的所有内容
* `vital_log.txt`  一些关键信息，包含MD5 校验数据，仿真时间等
* `README.MD`      本文件

在脚本目录下，首先执行命令：

```bash
    sudo chown YourAccountName *
    chmod u+x *.py
```

上面两条命令是为了防止文件权限没有在传输中丢失，添加执行权限，从而为后面的操作打下基础

然后打开`pose.json`文件，修改你需要的目标信息

接着执行以下指令：

```bash
./main.py
# 或者：
python2 main.py
```

需要注意以下几点：
1. 一定要在 Linux 平台运行（仅在 Ubuntu 18.04 上进行了测试）
2. 一定要使用普通用户执行本脚本，即不要使用`sudo ./main.py`
3. 一定要使用 Python2 执行（ROS Melodic 不支持 Python 3）

脚本运行后，会对运行环境进行简单的检查，将 Python 2 的默认编码修改为 utf-8 （通过修改`/usr/lib/python2.7/sitecustomize.py`实现）

接着，脚本所在目录中 `resources` 文件夹下的所有内容首先会进行 MD5 校验，然后被拷贝到对应位置，具体见下：
* `ucar_plane` 文件夹包含了图标文件，将会被移动到`~/.gazeboo/models`目录（如路径不存在将会自动创建）
* `gazebo_pkg` 文件夹包含了仿真模型，将会被移动到`~/iflytek_gazebo_ws/src`目录（如路径不存在将会自动创建）,然后自动在`~/iflytek_gazebo_ws`下执行`catkin_make`指令，并自动修改`.bashrc`文件，配置对应的环境变量

上述步骤结束后，脚本将会提醒用户是否安装如下两个依赖包：

* ros-melodic-teleop-twist-keyboard
* ros-melodic-rqt-graph

如果你的电脑上已经安装了这些内容，可按下“Enter”快速跳过，如果不确定是否已经安装，或者希望更新上述包，可以回答“Y”并回车

之后，本脚本将自动打开五个标签页，并执行如下命令

```bash
roscore
sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py
sleep 5; roslaunch gazebo_pkg race.launch
sleep 5; rviz
sleep 5; roslaunch ucar_nav only_navigation.launch
```

具体执行哪些命令可以按需在 `main.py` 修改，这里不再赘述。后四条命令中`sleep 5`的目的是保证其他程序在`roscore`启动完成后才被执行。

全部命令执行完成后，命令行提示“上述命令是否已全部在其他窗口正确执行？ [Y/n]”，这里作出肯定回答

提示“是否开始比赛计时？（务必等待所有软件启动完成后开始！）”，这里继续作出肯定回答，仿真开始，按屏幕上指示进行即可，仿真期间每 30s 打印一次 Topic list。
