

#### 变量
   - `operating_mode_`: 表示当前操作模式（例如，MANUAL）。
   - `action_client_`: 用于与 "/engineer_middleware/move_steps" 动作服务器交互的动作客户端。
   - 包含各种用于控制机器人行为的事件处理程序和参数。

#### 初始化
   - 构造函数初始化了ROS发布器、订阅器和其他组件。
   - 在继续之前等待中间件服务器启动。
#### 事件处理
   - 处理各种事件，如按钮按下、操纵杆移动和鼠标事件。
   - 事件触发机器人的特定动作，如发送指令到底盘或夹爪。

#### 速度模式配置
   - `changeSpeedMode` 方法根据所选的速度模式（LOW、NORMAL、FAST、EXCHANGE）调整速度和陀螺仪比例。

#### 键盘输入
   - `checkKeyboard` 方法处理来自 `rm_msgs::DbusData` 消息的键盘输入。

#### 反馈
   - 处理反馈、结果以及对发送到中间件动作服务器的操作取消的方法。

#### **伺服和速度控制:**
   -  `updateServo` 和 `updateRc` 

#### **云台控制:**
    - 用于在不同模式下控制云台的方法

####  **鼠标事件:**
    - 诸如 `mouseLeftRelease` 和 `mouseRightRelease` 之类的方法处理由鼠标释放按钮触发的事件。

#### **其他输入处理:**
    - `ctrlQPress`、`ctrlWPress` 处理特定键盘输入触发的事件。

#### **UI发布:**
    - `engineer_ui_pub_` 发布有关工程师UI的信息。

#### **动作执行:**
    - `runStepQueue` 方法向动作服务器发送目标以执行特定的步骤队列。

