## Robot Hardware Interface 机器人硬件接口



### RmRbotHWSim 
  

#### 注册 ROS 控制器接口：
```cpp
   gazebo_ros_control::DefaultRobotHWSim::registerInterface(&robot_state_interface_);
```

   注册了 ROS 控制器的接口,方便交互
#### 注册 IMU 传感器接口：
```cpp
   gazebo_ros_control::DefaultRobotHWSim::registerInterface(&imu_sensor_interface_);
```
#### 自定义 IMU 传感器接口：
```cpp
   gazebo_ros_control::DefaultRobotHWSim::registerInterface(&rm_imu_sensor_interface_);
```

#### 解析 IMU 
```cpp
   XmlRpc::XmlRpcValue xml_rpc_value;
   if (!model_nh.getParam("imus", xml_rpc_value))
     ROS_WARN("No imu specified");
   else
     parseImu(xml_rpc_value, parent_model);
```

   - 获取 IMU 的配置信息，如果没有配置信息则发出警告。
   - 调用 `parseImu` 函数来解析配置信息。

#### gazebo
```cpp
   world_ = parent_model->GetWorld();  // For gravity
```
   获取仿真世界的对象以及获取重力等信息。

#### 切换 IMU 状态
```cpp
   switch_imu_service_ = model_nh.advertiseService("switch_imu_status", switchImuStatus);
```

   切换 IMU 状态(禁用或启用)


这是 `RmRobotHWSim` 类的 `readSim` 函数。以下是有关这个函数的解释：

#### 读取仿真数据信息
```cpp
   gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
 ```

#### IMU 数据读取
   进行数据读取。这包括读取姿态、角速度和线性加速度。

#### 命令清零

```cpp
   // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
   for (auto& cmd : joint_effort_command_)
     cmd = 0;
   for (auto& cmd : joint_velocity_command_)
     cmd = 0;
```

   将关节的力和速度命令都设置为零，以避免在没有加载控制器时出现不稳定的软限位振荡。



###  parseImu

#### 遍历 IMU 数据：

   对传递进来的 IMU 数据进行遍历。

#### 核对获取信息

 - `frame_id`
 - `orientation_covariance_diagonal`
 - `angular_velocity_covariance`     
 - `linear_acceleration_covariance`

#### 获取指针：**

   获取 IMU 数据中指定的 `frame_id` 对应的链接指针。如果链接指针为 `nullptr`，则发出警告并继续下一次循环。

####  解析协方差矩阵数据

#### 存储 IMU 数据

   将解析得到的 IMU 数据存储到 `imu_datas_` 中。

#### 注册接口：

   使用解析得到的数据创建对象，并将其注册


 