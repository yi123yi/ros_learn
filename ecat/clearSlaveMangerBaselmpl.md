###ClearSlaveManagerBase

```cpp
//传入的参数初始化从设备管理器，并在独立模式下创建异步工作线程。
template <typename SlaveType>
ClearSlaveManagerBase<SlaveType>::ClearSlaveManagerBase(bool standalone, bool installSignalHandler, double timeStep)
    : standalone_(standalone), installSignalHandler_(installSignalHandler), timeStep_(timeStep), shutdownRequested_(false) {
// 判断是独立运行模式，创建并初始化异步工作线程
  if (standalone_) {
// 创建名为 "UpdateWorker" 的异步工作线程，时间步长为 timeStep_
    updateWorker_ = std::make_shared<any_worker::Worker>("UpdateWorker", timeStep_,
                                                         std::bind(&ClearSlaveManagerBase::updateWorkerCb, this, std::placeholders::_1));
  }
}
```
###ClearSlaveManagerBase
```cpp

template <typename SlaveType>
ClearSlaveManagerBase<SlaveType>::~ClearSlaveManagerBase() {
  if (installSignalHandler_) {
 // 解绑所有处理程序
    signal_handler::SignalHandler::unbindAll(&ClearSlaveManagerBase<SlaveType>::handleSignal, this);
  }
}
```
###setTimeStep
```cpp
//设置从设备管理器的时间步长，并在独立运行模式下更新异步工作线程的时间步长
template <typename SlaveType>
void ClearSlaveManagerBase<SlaveType>::setTimeStep(double timeStep) {
// 更新时间步长   
  timeStep_ = timeStep;
//如果为真，更新时间步长
  if (standalone_) {
    updateWorker_->setTimestep(timeStep_);
  }
}
```
###slaveExists
```cpp
//检查是否存在具有给定名称的从设备
template <typename SlaveType>
bool ClearSlaveManagerBase<SlaveType>::slaveExists(const std::string& name) const {
  for (const auto& slave : slaves_) {
    if (slave->getName() == name) {
      return true;
    }
  }
  return false;
}

```

###addSlave
```cpp
template <typename SlaveType>
bool ClearSlaveManagerBase<SlaveType>::addSlave(const SlavePtr& slave) {
  if (slaveExists(slave->getName())) {
  // 检查是否存在相同名称的从设备，如果存在则返回 false
    return false;
  }
// 将新的从设备添加到列表中
  slaves_.push_back(slave);
 // 返回 true 表示成功添加从设备
  return true;
}

```

###getSlave
```cpp
//根据给定的名称查找并返回从设备管理器中的从设备
template <typename SlaveType>
std::shared_ptr<SlaveType> ClearSlaveManagerBase<SlaveType>::getSlave(const std::string& name) const {
// 遍历从设备列表，寻找具有给定名称的从设备
  for (const auto& slave : slaves_) {
    if (slave->getName() == name) {
      return slave;
    }
  }
  // 如果未找到匹配的名称，返回 nullptr 表示没有找到相应的从设备
  return nullptr;
}

```


###getNamesOfSlaves
```cpp
//获取从设备管理器中所有从设备的名称
template <typename SlaveType>
std::vector<std::string> ClearSlaveManagerBase<SlaveType>::getNamesOfSlaves() const {
//// 创建一个存储从设备名称的字符串向量
  std::vector<std::string> names;
  for (const auto& slave : slaves_) {
  // 遍历从设备列表，将每个从设备的名称添加到向量中    
    names.push_back(slave->getName());
  }
 // 返回存储从设备名称的字符串向量
  return names;
}

```


###startup
```cpp
//启动从设备管理器
template <typename SlaveType>
bool ClearSlaveManagerBase<SlaveType>::startup() {
// 使用锁确保多线程下启动
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);

  // 用于标记启动是否成功
  bool success = true;
  // 如果为真,绑定处理信号的回调函数
  if (installSignalHandler_) {
    signal_handler::SignalHandler::bindAll(&ClearSlaveManagerBase<SlaveType>::handleSignal, this);
  }
//  如果为真，启动总线通信
  if (standalone_) {
    success &= busManager_->startupCommunication();
    if (!success) {
      return success;
    }
  }
  // 遍历从设备列表，将从设备切换到操作模式    
  for (const auto& slave : slaves_) {
// 等待从设备切换到模式
    if (!slave->waitForState(EC_STATE_SAFE_OP, 50, 0.05)) {
      MELO_ERROR_STREAM(slave->getName() << " not in safe op after startup!");
    }
  // 将从设备切换到模式
    slave->setState(EC_STATE_OPERATIONAL);
   // 等待从设备切换到模式
    success &= slave->waitForState(EC_STATE_OPERATIONAL, 50, 0.05);
  }

  if (standalone_) {
    success &= updateWorker_->start(99);
  }
// 处于运行状态，且无关闭请求
  isRunning_ = true;
  shutdownRequested_ = false;

  if (!success) MELO_ERROR("[ClearSlaveManagerBase::startup] Startup not successful.");
  return success;
}


```


###update
```cpp
template <typename SlaveType>
bool ClearSlaveManagerBase<SlaveType>::update() {
//更新通信管理器以读取从设备发送的消息。
  updateCommunicationManagerReadMessages();
// 处理读取的数据
  updateProcessReadings();
  // 发送暂存的命令
  updateSendStagedCommands();
// 更新通信管理器写入消息
  updateCommunicationManagerWriteMessages();
  return true;
}

```


###requestShutdown
```cpp
template <typename SlaveType>
void ClearSlaveManagerBase<SlaveType>::requestShutdown() {
 // 使用锁保护
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  // 标记管理器处于非运行状态，且有关闭请求
  isRunning_ = false;
  shutdownRequested_ = true;
  // 如果是独立运行模式，停止线程，并关闭总线通信
  if (standalone_) {
    updateWorker_->stop();
    busManager_->shutdownAllBuses();
  } else {
//历从设备列表，调用每个从设备的关闭函数
    for (const auto& slave : slaves_) {
      slave->shutdown();
    }
  }

  MELO_INFO("[ClearSlaveManagerBase::requestShutdown] Shutdown.");
}

```


###stageCommands
```cpp
template <typename SlaveType>
template <typename CommandType>
void ClearSlaveManagerBase<SlaveType>::stageCommands(const std::vector<CommandType>& commands) {
  // 遍历命令列表，将每个命令存储到对应的从设备中
  for (size_t i = 0; i < commands.size() && i < slaves_.size(); ++i) {
    slaves_[i]->stageCommand(commands[i]);
  }
}

```
