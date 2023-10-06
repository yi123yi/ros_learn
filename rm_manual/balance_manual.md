## 控制平衡车


#### 构造函数`BalanceManual` 类
 - 初始化了ROS节点句柄
 - 继承 `ChassisGimbalShooterCoverManual` 类
 - 初始化了`balance_cmd_sender_`，并设置了平衡状态为 `NORMAL`。

#### 参数读取
   - `flank_frame_` 
   - `reverse_frame_`
   - `balance_dangerous_angle_` 
#### 命令发送 `sendCommand` 函数
  - 用于发送底盘、速度、云台和平衡车的命令。
  - 根据`flank_`、`reverse_`、`supply_` 等状态，设置相应的命令。

#### `checkKeyboard` 函数
   检查键盘输入根据更新相应的状态。

#### 遥控器更新 `updateRc` 函数
  用于更新遥控器输入

#### 右侧开关按下处理 
 `rightSwitchDownRise` 和 `rightSwitchMidRise` 用于处理右侧开关的按下事件，设置平衡状态和底盘模式。

#### CTRL+Z 键按下处理
 `ctrlZPress` 用于处理CTRL+Z键的按下事件，根据供电状态设置平衡状态和底盘模式。

#### SHIFT 键按下和释放
 处理`shiftPress` 和 `shiftRelease` 用于处理SHIFT键的按下和释放事件，设置底盘模式。

#### V 键按下处理`vPress` 
 用于处理V键的按下事件，更新底盘的安全功率。

#### B 键按下处理 `bPress` 
  用于处理B键的按下事件，更新底盘的安全功率。

#### W、S、A、D 键按下处理 
`wPress`、`sPress`、`aPress`、`dPress` 用于处理W、S、A、D键的按下事件，根据侧翼状态设置底盘的安全功率。

#### W、S、A、D 键按住处理
`wPressing`、`sPressing`、`aPressing`、`dPressing` 用于处理W、S、A、D键的按住事件，根据侧翼状态设置底盘的线性速度。

#### C 键按下处理`cPress` 
 用于处理C键的按下事件，切换底盘控制模式。

#### CTRL+X 键按下处理`ctrlXPress`
 用于处理CTRL+X键的按下事件，切换平衡状态。

#### `balanceStateCallback`
  根据平衡角度和状态更新底盘的功率限制和判断是否进行自动倒下纠正

####`modeNormalize`
  用于将平衡状态归一化为正常状态

#### `modeFallen` 
用于处理平衡状态为倒下状态的事件

