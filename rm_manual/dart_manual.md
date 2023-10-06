## 机器人手动控制系统

#### **函数:**
```cpp
   DartManual::DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
```
   - 获取参数，包括`launch_id`、`trigger_position`、`dart_list`和`targets`。
   - 调用`getList`方法解析这些参数并存储在`dart_list_`和`target_position_`中。
   - 初始化了与机械臂相关的控制对象（`yaw_sender_`、`pitch_sender_`等）和其他成员变量。

#### `getList`
   - 构建了一个`Dart`对象列表（`dart_list_`）和目标位置的映射（`target_position_`）。

#### `run`
   - 更新`trigger_calibration_`和`gimbal_calibration_`对象。

#### 事件处理:
   - 使用`boost::bind`将特定函数与事件关联，例如左开关上、左开关中、左开关下等。

#### `sendCommand`
   - 发送控制命令到摩擦轮和扳机等组件。

####`updateRc`和`updatePc`
   - 处理遥控器和PC端的输入，更新机械臂的位置。

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

#### recordPosition
    - 记录机械臂的位置。

#### wheelClockwise和wheelAntiClockwise
  - 处理方向盘的事件，例如切换机械臂的微动和正常模式，切换机械臂的工作状态等。
