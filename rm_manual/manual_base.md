

#### **构造函数：**
```cpp
   ManualBase::ManualBase(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
```
   初始化了 `ManualBase` 类，接受两个 `ros::NodeHandle` 引用，分别用于处理主题和裁判系统主题。

#### **回调函数：**
   - `jointStateCallback`: 处理关节状态主题的回调函数。
   - `actuatorStateCallback`: 处理执行器状态主题的回调函数，更新执行器状态的时间戳。
   - `dbusDataCallback`: 处理遥控器数据主题的回调函数，根据遥控器数据更新相关状态。
   - `trackCallback`: 处理目标追踪数据主题的回调函数。
   - `gameRobotStatusCallback`: 处理比赛机器人状态主题的回调函数，更新机器人状态和电源状态。
   - `powerHeatDataCallback`: 处理电源和热量数据主题的回调函数，更新底盘功率和裁判系统最后一次通信时间。

#### 主循环函数

   - `checkReferee` 来更新裁判系统状态       
   - `controller_manager_.update` 更新控制器状态。

####  声明
   - `updateActuatorStamp`: 更新执行器状态的时间戳。
   - `updateRc`: 根据遥控器数据更新状态（如果操作模式为 RC）。
   - `updatePc`: 根据遥控器数据更新状态（如果操作模式为 PC）。
   - `remoteControlTurnOff`: 关闭遥控器控制模式。
   - `remoteControlTurnOn`: 打开遥控器控制模式。
   - `robotRevive`: 机器人复活。
   - `robotDie`: 机器人死亡。

