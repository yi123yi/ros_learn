## dbus.cpp

#### 主函数：
   - `main` 函数使用"rm_dbus"作为ROS节点的名称进行初始化。
   - 创建了名为 `dbus_node` 的 `DBusNode` 类的实例。
   - 程序以60Hz的速率进入循环，在循环中以同样的速率调用 `DBusNode` 实例的 `run` 方法。
   - 循环会在ROS停止时退出。

#### DBusNode 类:
   - **构造函数 (`DBusNode::DBusNode()`):**
     - 初始化ROS节点句柄 (`nh_`)。
     - 为类型为 `rm_msgs::DbusData` 的主题“dbus_data”创建了一个队列大小为1的发布器。
     - 从ROS参数服务器中读取串口参数或设置默认值（"/dev/usbDbus"）。
     - 使用指定的串口初始化 `Dbus` 类的实例 (`dbus_`)。

#### run` 方法 (`DBusNode::run()`):**
     - 调用 `Dbus` 实例的 `read` 方法从dbus中读取数据。
     - 调用 `Dbus` 实例的 `getData` 方法检索dbus数据。
     - 在主题"dbus_data"上发布dbus数据


## dbus_node


#### DBusNode 
```cpp
   DBusNode::DBusNode()
   {
     dbus_pub_ = nh_.advertise<rm_msgs::DbusData>("dbus_data", 1);
     nh_.param<std::string>("serial_port", serial_port_, "/dev/usbDbus");
     dbus_.init(serial_port_.data());
   }
```
       1.初始化 ROS 发布器（`dbus_pub_`）
     
       2.将 D-Bus 数据发布到名为 "dbus_data" 的 ROS 话题上。此外，从 ROS 参数服务器中获取串口参数并使用 
     
       3.初始化。

5. **DBusNode 类的运行函数：**
```cpp
   void DBusNode::run()
   {
     dbus_.read();
     dbus_.getData(dbus_cmd_);
     dbus_pub_.publish(dbus_cmd_);
   }
```
   - `dbus_.read` 从串口读取 D-Bus 数据
   - 通过 `dbus_.getData` 将其转换为 `rm_msgs::DbusData` 类型的消息
   - 通过 `dbus_pub_.publish` 发布到 ROS 话题中。


