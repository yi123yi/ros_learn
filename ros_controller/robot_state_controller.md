## robotstatecontroller

#### RobotStateController的初始化：
   - init方法用于初始化控制器。
   - 从控制器参数服务器中获取参数，如发布频率、是否使用静态变换广播器等。
   - 创建一个 `tf2_ros::Buffer` 对象（`tf_buffer_`），用于存储TF变换信息。
   - 创建一个 `rm_control::RobotStateHandle` 对象（`robot_state_handle`），并将其注册到机器人硬件接口中。
   - 初始化TF广播器（`tf_broadcaster_` 和 `static_tf_broadcaster_`），以及用于订阅TF消息的ROS话题（`tf_sub_` 和 `tf_static_sub_`）。

#### 机器人模型初始化：
   - 使用URDF模型描述初始化机器人模型。
   - 通过 `kdl_parser` 库将URDF模型转换为KDL（Kinematic Description Language）树结构。
   - 递归添加机器人的各个关节（`segments_`）和固定连接（`segments_fixed_`）到相应的映射中。
   - 还创建了一个 `mimic_` 映射，用于跟踪关节的模仿信息。

#### `update` 方法：
   - `update` 方法用于更新控制器的状态。
   - 如果时间向后移动（通常因为ROS时钟重置），则清除TF缓冲区。
   - 遍历机器人的各个关节和固定连接，计算并发布TF变换信息。
   - 发布TF信息的频率由 `publish_rate_` 控制，通常是以一定的频率发送。

#### 辅助函数 `stripSlash`：

   - 用于去掉字符串开头的斜杠字符。

#### `tfSubCallback` 和 `staticSubCallback` 方法：
   - 用于处理接收到的TF消息，将其写入缓冲区。

#### 通过 `PLUGINLIB_EXPORT_CLASS` 宏将 `RobotStateController` 类声明为ROS插件，使其可用于ROS控制器管理。

