## 机器人手动控制系统

#### **函数:**
```cpp
   DartManual::DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
```
   - 获取参数，包括`launch_id`、`trigger_position`、`dart_list`和`targets`。
   - 调用`getList`方法解析这些参数并存储在`dart_list_`和`target_position_`中。
   - 初始化了与机械臂相关的控制对象（`yaw_sender_`、`pitch_sender_`等）和其他成员变量。

#### `getList`
   1. 函数名为 `DartManual::getList`。

   2. 函数接受四个参数：`darts`、`targets`、`launch_id` 和 `trigger_position`，它们都是 XmlRpc::XmlRpcValue 类型的参数。

   3. 函数通过一个循环遍历 `darts` 数据，并对每个 `dart` 执行一系列操作。

   - 使用 `ROS_ASSERT` 来确保 `dart` 具有 "param" 和 "id" 成员。
   - 使用 `ROS_ASSERT` 来确保 "param" 是一个数组类型（TypeArray），"id" 是一个整数类型（TypeInt）。
   - 循环，遍历 `launch_id` 的四个元素，寻找匹配的 `dart`，并将相关信息存储在 `Dart` 结构中。
   - 最后，将 `Dart` 结构存储在 `dart_list_` 容器中，使用 `i` 作为键。

   函数循环遍历 `targets` 数据，执行类似的 `ROS_ASSERT` 检查，然后将目标的位置信息存储在 `target_position_` 容器中。

#### `run`
   - 更新`trigger_calibration_`和`gimbal_calibration_`对象。

#### 事件处理:
   - 使用`boost::bind`将特定函数与事件关联，例如左开关上、左开关中、左开关下等。

#### `sendCommand`
   - 发送控制命令到摩擦轮和扳机等组件。

####`updateRc`和`updatePc`
   - 处理遥控器和PC端的输入，更新机械臂的位置。
   
##### updatePc

   - 函数接受一个rm_msgs::DbusData::ConstPtr 类型的参数，通常是一个指向 Dbus 数据的常量指针.
   - 函数的目的是根据传入的数据更新控制系统的状态和执行操作.
   - 函数首先调用 `ManualBase::updatePc(dbus_data)`，可能用于在基类中执行一些通用操作。
   - velocity_threshold检查 `pitch_velocity_` 和 `yaw_velocity_` 是否小于 `velocity_threshold`。如果两者都小于该阈值，则设置 `move_state_` 为 `STOP`，否则设置为 `MOVING`。
   -  如果 `game_progress_` 等于 `rm_msgs::GameStatus::IN_BATTLE`，则会进入一系列条件分支，其中 `auto_state_` 的不同值会导致不同的操作
   - 检查 `dart_door_open_times_` 和 `dart_launch_opening_status_` 的条件。当 `dart_door_open_times_` 和 `dart_launch_opening_status_` 满足一定条件时，将执行一些操作。
   - 根据 launch_state_和dart_launch_opening_status_ 的值，会设置 `trigger_sender_` 的点（point），以控制触发器的操作。
   - 如果 `game_progress_` 不等于 `rm_msgs::GameStatus::IN_BATTLE`，则将 `friction_right_sender_` 和 `friction_left_sender_` 的点都设置为零，停止摩擦力控制。

#### `checkReferee`
   - 检查裁判系统的状态。

#### `remoteControlTurnOn`
   - 打开遥控器控制，停止摩擦轮和扳机的控制器并重置它们。

#### leftSwitchDownOn和leftSwitchMidOn
    - 处理左开关的事件，例如控制扳机、摩擦轮等。

#### rightSwitchDownOn和rightSwitchUpRise
    - 处理右开关的事件，例如控制摩擦轮和机械臂的位置。

#### move
    - 控制机械臂的运动。

####triggerComeBackProtect和waitAfterLaunch
    - 扳机回位保护和发射后等待。

#### launchTwoDart和getDartFiredNum
    - 控制发射逻辑，获取发射的飞镖数量。
    - `if (dart_fired_num_ < 4)`: 这个条件检查 `dart_fired_num_` 是否小于 4。如果满足条件，表示可以继续操作。
    - `if (dart_fired_num_ - initial_dart_fired_num_ < 2)`: 这个条件检查 `dart_fired_num_` 与 `initial_dart_fired_num_` 的差是否小于 2。如果满足条件，表示发射两个飞镖中的一个。
    - `if (dart_fired_num_ - initial_dart_fired_num_ == 1)`: 这个条件检查 dart_fired_num_与initial_dart_fired_num_ 的差是否等于 1。如果满足条件，表示发射第二个飞镖后等待 2.0 秒。
   - `dart_fired_num_ - initial_dart_fired_num_` 不等于 1，将设置 `trigger_sender_` 的点（point）为 `upward_vel_`。控制飞镖发射的速度。
   - 如果 `dart_fired_num_` 不小于 4，将设置 `trigger_sender_` 的点为 0，表示不发射飞镖。

####waitAfterLaunch

1. 函数接受一个 `time` 参数，表示等待的时间

2. 函数的目的是等待一段时间，然后执行特定的操作。

3. 首先，函数检查 `has_stopped` 是否为 `false`。如果 `has_stopped` 为 `false`，它会记录当前时间为 `stop_time_`。

4. 然后，函数检查当前时间与 `stop_time_` 的差是否小于指定的 `time`（以 `ros::Duration` 表示的时间间隔）。如果差小于 `time`，则执行以下操作：
   - 设置 `trigger_sender_` 的点（point）为 0，可能是用于停止发射飞镖。
   - 将 `has_stopped` 设置为 `true`，以表示已经停止。

 5.如果当前时间与 `stop_time_` 的差不小于指定的 `time`，则设置 `trigger_sender_` 的点为 `upward_vel_`。这可能是用于控制飞镖的垂直速度。
  飞镖发射后等待一段指定的时间，然后执行一系列操作，包括停止发射和控制垂直速度。这可以用于调整飞镖的行为和轨迹


#### recordPosition
    - 记录机械臂的位置。

#### wheelClockwise和wheelAntiClockwise
  - 处理方向盘的事件，例如切换机械臂的微动和正常模式，切换机械臂的工作状态等。



####std::unordered_map<int, Dart> dart_list_{};
这行代码定义了一个名为 `dart_list_` 的 C++ 变量，它是一个无序映射

1. `std::unordered_map` 是 C++ 标准库中的一种数据结构，用于创建关联数组，其中每个元素都有一个唯一的键（key）与之关联。

2. `<int, Dart>`：这表示键是整数类型（`int`），而值是 `Dart` 类型的对象。在这个映射中，整数键将与 `Dart` 对象相关联。

3. `dart_list_{}`：这是对 `dart_list_` 变量的初始化。通过使用 `{}`，您创建了一个空的无序映射。这意味着 `dart_list_` 是一个初始为空的无序映射，可以在程序中用来存储与整数键相关联的 `Dart` 对象。

这种无序映射允许您使用整数键（例如，`int`）来查找和访问相关的 `Dart` 对象。通常，这种数据结构用于建立从一组唯一键到相应值的映射关系，以便快速查找和检索值。