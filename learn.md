底盘控制器的实现:
1.'SimpleChassisController'  包含了控制器的所有定义。
2.'SimpleChassisController'类实现了 'controller_interface::ControllerBase'接口
3.'init' 函数是在控制器初始化时调用的，用于设置底盘控制器。在这里，通过 `effort_joint_interface` 获取了四个轮子的关节句柄，分别是 'front_left_joint_'、'front_right_joint_'、'back_left_joint_'、'back_right_joint_'。
4. 'update'函数是在控制器更新时调用的，用于执行实际的底盘控制逻辑。
5. 'cmd'数组包含了六种运动状态下的力矩命令，分别是前进、后退、左转、右转、顺时针旋转、逆时针旋转。
6. 'state' 变量表示当前底盘的状态，通过修改这个状态可以切换不同的运动模式。
7. 'last_change_' 记录了上一次状态切换的时间，确保两次状态切换之间有足够的时间间隔。


roslaunch文件
"rosparam"通过加载yaml配置文件将参数加载到控制器

 `load_chassis` 设置为 `true`，表示加载底盘（chassis）。
 `roller_type` 设置为 `realistic`，指定轮子的类型。
 `paused` 设置为 `true`，表示仿真环境一开始是暂停状态。

`<rosparam>` 标签部分：这里使用 `<rosparam>` 标签来加载控制器配置文件 `controllers.yaml` 到ROS参数服务器中，并使用 `command="load"` 命令加载参数。这些参数可能包含控制器的配置信息。
通过 `<node>` 标签启动 `controller_manager` 的 `spawner` 节点，用于启动控制器。
`args`列出了要启动的控制器的名称列表，包括 `simple_chassis_controller` 和`joint_state_controller`。

hero_chassis_controller.cpp

这是一个ROS控制器（controller），使用PID控制来控制四个轮子，实现底盘（chassis）的运动。下面是对代码的一些关键部分的解释：

1. **初始化（init）函数：**
   - **获取参数：** 从ROS参数服务器中获取底盘的一些参数，如 `Wheel_Track`、`Wheel_Base` 等。
   - **获取关节句柄：** 通过硬件接口 `hardware_interface::EffortJointInterface` 获取四个轮子的关节句柄，即 `front_left_joint_`、`front_right_joint_`、`back_left_joint_` 和 `back_right_joint_`。
   - **初始化PID控制器：** 使用 `control_toolbox::Pid` 类初始化四个PID控制器，分别为 `pid1_controller_`、`pid2_controller_`、`pid3_controller_` 和 `pid4_controller_`。

2. **更新（update）函数：**
   - **获取时间和周期：** 获取当前时间和时间周期。
   - **获取实际速度：** 获取四个轮子的实际速度。
   - **计算底盘速度：** 根据输入的线速度和角速度计算底盘的速度，其中 `compute_mecvel()` 函数用于计算。
   - **广播变换（Transform broadcast）：** 通过TF库（Transform Library）广播 "odom" 到 "base_link" 的变换。
   - **发布里程计信息（Odometry publish）：** 发布底盘的里程计信息到 "odom" 话题。
   - **PID控制器计算：** 计算四个轮子的速度误差，并使用PID控制器计算对应的控制命令。
   - **控制器状态发布：** 每隔10个周期发布一次控制器的状态信息到 "state" 话题。

3. **获取底盘运动指令：**
   - **通过`cmd_vel`话题订阅底盘的线速度和角速度指令。**

4. **其他函数：**
   - **`compute_vel_rte()` 函数：** 计算期望速度，用于根据期望速度和实际速度计算PID误差。
   - **`compute_chassis_velocity()` 函数：** 计算底盘的线速度和角速度。
   - **`Transform_broadcast()` 函数：** 广播底盘的变换信息。
   - **`Odometry_publish()` 函数：** 发布底盘的里程计信息。

5. **析构函数：**
   - **在析构函数中关闭订阅器和发布器。**

6. **导出控制器：**
   - **使用 `PLUGINLIB_EXPORT_CLASS` 宏导出控制器类，以便ROS能够识别和加载该控制器。**

此控制器的主要目标是通过订阅底盘的运动指令（线速度和角速度），使用PID控制器控制四个轮子的速度，同时发布底盘的里程计信息。这有助于在ROS中实现底盘的运动控制和定位功能。



keyboard


1. **键盘输入处理：** 通过 `getch` 函数实现非阻塞键盘输入，捕捉用户按下的键。

2. **控制映射：** 定义了两个映射表 `moveBindings` 和 `speedBindings`。
    - `moveBindings` 映射了键盘上的按键到机器人的运动方向，包括前进、后退、左移、右移等。
    - `speedBindings` 映射了键盘上的按键到机器人的速度变化，包括线速度和角速度的增加和减少。

3. **ROS节点初始化：** 初始化ROS节点和 `cmd_vel` 的发布者。

4. **Twist消息更新：** 根据键盘输入更新 `geometry_msgs::Twist` 消息，包括线速度和角速度。

5. **循环发布Twist消息：** 在循环中发布更新后的Twist消息，实现实时控制机器人运动。

6. **退出程序：** 当用户按下Ctrl+C时，退出程序。

此程序允许通过键盘输入实时控制机器人的运动，是一个用于调试和测试机器人底盘的工具。通过按不同的键，可以控制机器人的运动方向、速度等参数。这对于机器人底盘的调试和验证非常有用。




cmakelists


cmake_minimum_required(VERSION 3.10)   声明了项目所需的CMake最低版本。
project(hero_chassis_controller) 设置项目名称。

set(CMAKE_CXX_STANDARD 14)  设置C++标准为C++14。

add_definitions(-Wall -Werror) 添加编译器标志，将所有警告视为错误，以确保更严格的代码规范。

find_package  查找catkin和其他所需的ROS软件包。

catkin_package  声明catkin软件包的依赖关系和其他配置信息。

include_directories   包含所需的头文件目录。

`add_library` 和 `add_executable`：
   - `add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)`：将src/${PROJECT_NAME}.cpp编译成库。
   - `add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)`：将src/teleop_twist_keyboard.cpp编译成可执行文件。

add_dependencies` 和 `target_link_libraries`
   - `add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})`：声明库的依赖关系。
   - `target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})`：指定库的链接库。

roslint_cpp()   调用`roslint_cpp()`来执行静态代码分析，确保代码符合ROS代码风格规范。


