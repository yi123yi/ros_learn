balance.launch

1. `<arg>` 标签：这些标签用于定义Launch文件的参数，这些参数可以在启动文件中使用。例如，`robot_type` 参数定义了机器人的类型，默认值是从环境变量 `ROBOT_TYPE` 中获取的。`load_controller` 和 `balance` 参数也有默认值。
2. `<rosparam>` 标签：这些标签用于从外部文件加载ROS参数。在这里，`dbus.yaml` 文件中的参数将被加载，并且根据 `robot_type` 的值选择相应的参数文件加载。
3. `<node>` 标签：这个节点用于启动 `rm_dbus` 包中的 `rm_dbus` 节点。这是一个ROS节点，可能用于处理机器人和其他设备之间的通信。
4. `<include>` 标签：这些标签用于包含其他Launch文件。在这里，有三个 `<include>` 标签，分别用于包含不同的Launch文件：
   - 第一个 `<include>` 标签包含了机器人硬件的Launch文件，根据 `HW_NAME` 参数选择加载 `rm_can_hw` 或 `rm_ecat_hw` 的Launch文件。
   - 第二个 `<include>` 标签包含了 `manual.launch` 的Launch文件，并且传递了 `balance` 参数的值。
   - 第三个 `<include>` 标签包含了 `referee.launch` 的Launch文件，用于启动与裁判系统相关的组件。


drone.launch


1. `<arg>` 标签：这些标签用于定义Launch文件的参数，它们可以在启动文件中使用。例如，`robot_type` 参数定义了机器人的类型，默认值是从环境变量 `ROBOT_TYPE` 中获取的。`load_controller`、`imu_trigger` 和 `hw_name` 参数也有默认值。

2. `<rosparam>` 标签：这些标签用于从外部文件加载ROS参数。在这里，`dbus.yaml` 文件和根据机器人类型选择的控制器参数文件将被加载。

3. `<node>` 标签：这个节点用于启动 `rm_dbus` 包中的 `rm_dbus` 节点。这是一个ROS节点，可能用于处理机器人和其他设备之间的通信。

4. `<include>` 标签：这些标签用于包含其他Launch文件。在这里，有几个 `<include>` 标签，分别用于包含不同的Launch文件：
   - 第一个 `<include>` 标签包含了机器人硬件的Launch文件，根据 `HW_NAME` 参数选择加载 `rm_can_hw` 或 `rm_ecat_hw` 的Launch文件。
   - 第二个 `<include>` 标签包含了 `camera.launch` 的Launch文件，并且传递了 `imu_trigger` 参数的值。
   - 第三个 `<include>` 标签包含了 `vision_loader.launch` 的Launch文件，用于加载与视觉相关的节点。
   - 第四个 `<include>` 标签包含了 `manual.launch` 的Launch文件，该文件可能包含手动控制节点的配置。
   - 第五个 `<include>` 标签包含了 `referee.launch` 的Launch文件，用于启动与裁判系统相关的组件。
   - 最后一个 `<include>` 标签包含了 `t265_all_nodes.launch` 的Launch文件，可能用于与 `vision_to_mavros` 相关的节点。


engineer.launch



1. `<arg>` 标签：这些标签用于定义Launch文件的参数，它们可以在启动文件中使用。例如，`load_hw` 参数用于决定是否加载硬件相关的组件，默认值为 `true`。`odom_tf` 参数用于决定是否发布里程计的 `map` 到 `odom` 的 `tf` 变换，默认值为 `true`。`hw_name` 参数用于指定硬件类型，默认值是从环境变量 `HW_NAME` 中获取的。

2. `<node>` 标签：这个节点用于启动 `static_transform_publisher`，它会发布一个静态的 `tf` 变换，将 `map` 坐标系和 `odom` 坐标系连接起来。这样可以方便地在RViz中查看机器人的位姿。

3. `<rosparam>` 标签：这个标签用于从外部文件加载ROS参数。在这里，`engineer.yaml` 文件将被加载，该文件包含机器人控制器的参数。

4. `<node>` 标签：这个节点用于启动 `rm_dbus` 包中的 `rm_dbus` 节点。这是一个ROS节点，可能用于处理机器人和其他设备之间的通信。

5. `<include>` 标签：这些标签用于包含其他Launch文件。在这里，有几个 `<include>` 标签，分别用于包含不同的Launch文件：
   - 第一个 `<include>` 标签根据 `load_hw` 参数的值选择是否加载硬件相关的Launch文件，`$(env HW_NAME).launch` 包含了机器人硬件的配置。
   - 第二个 `<include>` 标签包含了机械臂运动规划组件的Launch文件，可能包含与 MoveIt! 相关的配置。
   - 第三个 `<include>` 标签包含了机器人中间件的Launch文件，用于启动与通信相关的组件。
   - 第四个 `<include>` 标签包含了 `manual.launch` 的Launch文件，该文件可能包含手动控制节点的配置。
   - 第五个 `<include>` 标签包含了与机械臂相关的 `servo.launch` 的Launch文件，用于启动机械臂伺服控制相关的节点。

sentry.launch


参数设置：

    robot_type: 机器人类型，可选值为 "standard", "hero", "engineer"。通过环境变量 ROBOT_TYPE 设置，默认为空字符串。
    load_controller: 是否加载控制器，可选值为 "true" 或 "false"，默认为 "true"。
    hw_name: 硬件名称，可选值为 "rm_can_hw" 或 "rm_ecat_hw"。通过环境变量 HW_NAME 设置，默认为空字符串
    
    
    加载参数文件：
        dbus.yaml: 从 rm_dbus 包的 config 目录加载，用于配置 ROS 数据总线。
        $(env HW_NAME).launch: 根据环境变量 HW_NAME 加载不同的硬件配置文件。
        
     启动 rm_dbus 节点：
        rm_dbus 节点是 rm_dbus 包中的一个程序，用于处理 ROS 数据总线。
        
     加载 rm_fsm 节点：
        rm_fsm 包中的 load.launch 文件用于加载有关状态机的配置。  
     加载 referee.launch 文件：
        referee.launch 文件可能包含有关裁判系统的配置   
     启动激光雷达地面分割和导航相关组件：
        启动 LiO 激光雷达定位组件、线拟合地面分割、导航 Move Base 组件和 PCL 滤波器。 
        
        
 start.launch
   这是一个 ROS Launch 文件，用于配置和启动 ROS 节点和相关组件。以下是对该文件主要部分的解释：

1. **参数设置：**
    - `robot_type`: 机器人类型，可选值为 "standard", "hero", "engineer"。通过环境变量 `ROBOT_TYPE` 设置，默认为空字符串。
    - `load_controller`: 是否加载控制器，可选值为 "true" 或 "false"，默认为 "true"。
    - `hw_name`: 硬件名称，可选值为 "rm_can_hw" 或 "rm_ecat_hw"。通过环境变量 `HW_NAME` 设置，默认为空字符串。


2. **加载参数文件：**
    - `dbus.yaml`: 从 `rm_dbus` 包的 `config` 目录加载，用于配置 ROS 数据总线。
    - `rm_controllers/$(arg robot_type).yaml`: 从 `rm_config` 包的 `config` 目录加载，用于配置机器人控制器。


3. **启动 `rm_dbus` 节点：**
    - `rm_dbus` 节点是 `rm_dbus` 包中的一个程序，用于处理 ROS 数据总线。

4. **加载硬件配置文件：**
    - 根据环境变量 `HW_NAME` 加载不同的硬件配置文件。

5. **加载手动控制节点：**
    - `manual.launch` 文件可能包含手动控制节点的配置。

6. **加载裁判系统相关配置：**
    - `referee.launch` 文件可能包含与裁判系统相关的配置。

