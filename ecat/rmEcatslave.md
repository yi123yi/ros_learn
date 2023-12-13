##rmEcatSlave

### 构造函数和构造函数
```cpp
//用于从配置文件创建并返回相关指针,提供配置文件路径、名称和地址来地创建和配置对象。
   RmEcatSlave::SharedPtr RmEcatSlave::deviceFromFile(const std::string& configFile, const std::string& name, const uint32_t address);
```

### 状态控制
```cpp
//将指定的状态应用到与对象关联的EtherCAT从站,通过改变从站的状态，可以控制其行为
   void RmEcatSlave::setState(uint16_t state);
//用于在启动EtherCAT时等待从站进入特定的状态
   bool RmEcatSlave::waitForState(uint16_t state, unsigned int maxRetries, double retrySleep);
//等待从站进入指定的状态，以确保从站已经准备好接收配置信息。
//根据配置信息进行设置，以便从硬件读取数据。
//根据配置信息设置 GPIO 模式。用于设置从站的 GPIO 模式。
//设置 RxPdo 和 TxPdo 的大小。用于配置从站的过程数据对象。
   void RmEcatSlave::startup();

   void RmEcatSlave::shutdown();
```

### 更新数据读写
```cpp
void RmEcatSlave::updateWrite() {
  RxPdo rxPdo{};
  {
// 锁定确保在访问时没有其他线程修改它
    std::lock_guard<std::mutex> stagedCmdLock(stagedCommandMutex_);
//映射到RxPdo的成员
    rxPdo.controlword_ = controlwordToId(controlword_);
 // 将每个电机的目标扭矩和数字输出映射
    for (size_t i = 0; i < 8; ++i) {
      rxPdo.can0MotorCommnads_[i] = stagedCommand_.getTargetTorqueRaw(CanBus::CAN0, i + 1);
      rxPdo.can1MotorCommnads_[i] = stagedCommand_.getTargetTorqueRaw(CanBus::CAN1, i + 1);
    }
    rxPdo.digital_outputs_ = stagedCommand_.getDigitalOutputs();
  }

  // actually writing to the hardware
// 实际将RxPdo数据写入硬件
  bus_->writeRxPdo(address_, rxPdo);
}
```
```cpp
  void RmEcatSlave::updateRead() {
  TxPdo txPdo{};
  // reading from the bus
// 从总线上读取TxPdo数据
  bus_->readTxPdo(address_, txPdo);

  Statusword statusword;
 // 使用TxPdo中的数据设置它
  statusword.setRaw(txPdo.statusword_);
// 设置Reading对象的状态字和时间戳
  reading_.setStatusword(statusword);
  reading_.setStamp(bus_->getUpdateReadStamp());

  // Motors
// 设置电机数据和IMU数据
  for (size_t i = 0; i < 8; ++i) {
    reading_.setPosition(CanBus::CAN0, i + 1, txPdo.can0MotorPositions_[i]);
    reading_.setVelocity(CanBus::CAN0, i + 1, txPdo.can0MotorVelocities_[i]);
    reading_.setTorque(CanBus::CAN0, i + 1, txPdo.can0MotorCurrents_[i]);
    reading_.setTemperature(CanBus::CAN0, i + 1, txPdo.can0MotorTemperatures_[i]);
    reading_.setPosition(CanBus::CAN1, i + 1, txPdo.can1MotorPositions_[i]);
    reading_.setVelocity(CanBus::CAN1, i + 1, txPdo.can1MotorVelocities_[i]);
    reading_.setTorque(CanBus::CAN1, i + 1, txPdo.can1MotorCurrents_[i]);
    reading_.setTemperature(CanBus::CAN1, i + 1, txPdo.can1MotorTemperatures_[i]);
  }
  // IMUs
  // 设置IMU数据和数字输入
  if (statusword.isAngularVelocityUpdated(CanBus::CAN0)) {
    reading_.setAngularVelocity(CanBus::CAN0, txPdo.can0ImuAngularVelocity_);
  }
  if (statusword.isLinearAccelerationUpdated(CanBus::CAN0)) {
    reading_.setLinearAcceleration(CanBus::CAN0, txPdo.can0ImuLinearAcceleration_);
  }
  if (statusword.isAngularVelocityUpdated(CanBus::CAN1)) {
    reading_.setAngularVelocity(CanBus::CAN1, txPdo.can1ImuAngularVelocity_);
  }
  if (statusword.isLinearAccelerationUpdated(CanBus::CAN1)) {
    reading_.setLinearAcceleration(CanBus::CAN1, txPdo.can1ImuLinearAcceleration_);
  }
  // Digital inputs
// 设置Reading对象的数字输入
  reading_.setDigitalInputs(txPdo.digital_inputs_);
}

```

### 命令和读取暂存
```cpp
//将CAN0和CAN1上的所有电机的目标指令均为零
   void RmEcatSlave::stageZeroCommand();
//设置每个电机设置最大输出和扭矩
   void RmEcatSlave::stageCommand(const Command& command);

```

### 获取状态和读取
```cpp
//获取当前的成员变量
   Command RmEcatSlave::getStageCommand();
//获取当前的reading_成员变量
   Reading RmEcatSlave::getReading() const;
//参数传递
   void RmEcatSlave::getReading(Reading& reading) const;
```

### 配置文件加载
```cpp
//从指定的文件加载配置
   bool RmEcatSlave::loadConfigFile(const std::string& fileName);
//加载部分配置
   bool RmEcatSlave::loadConfigNode(const YAML::Node& configNode);
```

###其他功能
```cpp
   Configuration RmEcatSlave::getConfiguration() const;
//获取 EtherCAT 从站的状态字
   bool RmEcatSlave::getStatuswordViaSdo(Statusword& statusword);
//用于设置IMU触发
   void RmEcatSlave::setImuTrigger(CanBus bus, bool imuTrigger);
   void RmEcatSlave::setGpioModes(uint8_t mode);
```

