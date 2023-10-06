## 手动控制机器人的底盘、云台和射击装置



#### 初始化
- `ShooterCommandSender`、`CameraSwitchCommandSender`、用于控制镜头和图传的 
-  `JointPositionBinaryCommandSender`，以及用于不同键盘和鼠标输入的事件处理程序。

#### 事件处理 
`setRising`、`setEdge`、`setActiveHigh`进行事件处理，例如处理按键和释放。

#### 校准 
 用于射手和云台的校准机制，更新校准参数。


#### 回调函数 
- `gameRobotStatusCallback`
- `powerHeatDataCallback`
- `dbusDataCallback` 

#### 发送命令`sendCommand` 函数
   根据当前状态和输入发送命令，以控制底盘、云台、射手和其他组件。

#### 输入处理 checkKeyboard 函数
   负责根据键盘输入更新内部状态。

#### 状态检查 checkReferee 函数
  检查裁判系统的状态，并更新将要发布的某些信息。



