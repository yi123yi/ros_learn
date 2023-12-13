### getMotorNames
```cpp
//    遍历所有启用的电机，获取它们的名称。
std::vector<std::string> RmSlaveManager::getMotorNames() const {
  std::vector<std::string> names;
//获取与从设备关联电机读数
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
//获取启用的电机 ID
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
//遍历每个电机 ID
    for (const auto& id : ids) {
//获取相应电机的名称
      names.push_back(reading.getMotorName(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      names.push_back(reading.getMotorName(CanBus::CAN1, id));
    }
  }
  return names;
}

```

###getMotorPositions
```cpp
//获取所有启用的电机的位置信息
std::vector<double> RmSlaveManager::getMotorPositions() const {
  std::vector<double> positions;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
//获取相应电机的位置
      positions.push_back(reading.getPosition(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      positions.push_back(reading.getPosition(CanBus::CAN1, id));
    }
  }
  return positions;
}
```

###getMotorVelocities
```cpp
//获取所有启用的电机的速度信息
std::vector<double> RmSlaveManager::getMotorVelocities() const {
  std::vector<double> velocities;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
//获取相应电机的速度
      velocities.push_back(reading.getVelocity(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      velocities.push_back(reading.getVelocity(CanBus::CAN1, id));
    }
  }
  return velocities;
}
```

###getMotorTorque
```cpp
//获取所有启用的电机的扭矩信息
std::vector<double> RmSlaveManager::getMotorTorque() const {
  std::vector<double> torques;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
////获取相应电机的扭矩
      torques.push_back(reading.getTorque(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      torques.push_back(reading.getTorque(CanBus::CAN1, id));
    }
  }
  return torques;
}
```

###getMotorIsOnlines
```cpp
//获取所有启用的电机的在线状态
std::vector<bool> RmSlaveManager::getMotorIsOnlines() const {
  std::vector<bool> isOnlines;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      isOnlines.push_back(reading.getStatusword().isOnline(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      isOnlines.push_back(reading.getStatusword().isOnline(CanBus::CAN1, id));
    }
  }
  return isOnlines;
}
```

###getMotorNeedCalibrations
```cpp
//获取所有启用的电机是否需要校准
std::vector<bool> RmSlaveManager::getMotorNeedCalibrations() const {
  std::vector<bool> needCalibrations;
//获取从设备列表
  for (const auto& slave : getSlaves()) {
    const auto reading = slave->getReading();
//通过从设备的配置信息获取该电机是否需要校准的信息
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      needCalibrations.push_back(slave->getConfiguration().can0MotorConfigurations_.at(id).needCalibration_);
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      needCalibrations.push_back(slave->getConfiguration().can1MotorConfigurations_.at(id).needCalibration_);
    }
  }
  return needCalibrations;
}
```

###stageMotorCommands

```cpp
//为每个电机设置指令
void RmSlaveManager::stageMotorCommands(const std::vector<double>& commands) {
  auto commandItr = commands.begin();
  for (auto& slave : slaves_) {
    Command command;
    auto ids = slave->getReading().getEnabledMotorIds(CanBus::CAN0);
//设置电机目标命令
    for (const auto& id : ids) {
      command.setTargetCommand(CanBus::CAN0, id, *commandItr++);//获取下一个电机命令
      if (commandItr == commands.end()) {
        slave->stageCommand(command);
        return;
      }
    }
    ids = slave->getReading().getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      command.setTargetCommand(CanBus::CAN1, id, *commandItr++);
      if (commandItr == commands.end()) {
        slave->stageCommand(command);
        return;
      }
    }
    slave->stageCommand(command);
  }
}
```

###getImuNames
```cpp
//获取所有启用的IMU的名称
std::vector<std::string> RmSlaveManager::getImuNames() const {
  std::vector<std::string> names;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      names.push_back(reading.getImuName(bus));
    }
  }
  return names;
}
```

###getImuOrientations
```cpp
//获取所有启用的IMU的方向信息。
std::vector<double> RmSlaveManager::getImuOrientations() const {
  std::vector<double> orientations;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
//获取信息(四元数)
      double w = 1., x = 0., y = 0., z = 0.;
      reading.getOrientation(bus, w, x, y, z);
      orientations.push_back(x);
      orientations.push_back(y);
      orientations.push_back(z);
      orientations.push_back(w);
    }
  }
  return orientations;
}
```
###getImuLinearAccelerations
```cpp
//获取所有启用的IMU的线性加速度信息
std::vector<double> RmSlaveManager::getImuLinearAccelerations() const {
  std::vector<double> accelerations;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      double x = 0., y = 0., z = 0.;
      reading.getLinearAcceleration(bus, x, y, z);
      accelerations.push_back(x);
      accelerations.push_back(y);
      accelerations.push_back(z);
    }
  }
  return accelerations;
}
```

###getImuAngularVelocities
```cpp
//获取所有启用的IMU的角速度信息
std::vector<double> RmSlaveManager::getImuAngularVelocities() const {
  std::vector<double> angularVelocities;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      double x = 0., y = 0., z = 0.;
      reading.getAngularVelocity(bus, x, y, z);
      angularVelocities.push_back(x);
      angularVelocities.push_back(y);
      angularVelocities.push_back(z);
    }
  }
  return angularVelocities;
}
```

###getDigitalInputNames
```cpp
//获取所有数字输入的名称
std::vector<std::string> RmSlaveManager::getDigitalInputNames() const {
  std::vector<std::string> names;
  for (const auto& slave : slaves_) {
    auto configs = slave->getConfiguration().gpioConfigurations_;
    for (const auto& config : configs) {
      if (config.second.mode_ == 0) {
        names.push_back(config.second.name_);
      }
    }
  }
  return names;
}
```

###getDigitalInputs
```cpp
//状态
std::vector<bool> RmSlaveManager::getDigitalInputs() const {
  std::vector<bool> inputs;
  for (auto& reading : getReadings<rm_ecat::Reading>()) {
    auto ids = reading.getEnabledDigitalInputIds();
    for (const auto& id : ids) {
      inputs.push_back(reading.getDigitalInput(id));
    }
  }
  return inputs;
}
```

###getDigitalOutputNames
```cpp
//获取所有数字输出的名称
std::vector<std::string> RmSlaveManager::getDigitalOutputNames() const {
  std::vector<std::string> names;
//遍历每个从设备
  for (const auto& slave : slaves_) {
//获取输出配置
    auto configs = slave->getConfiguration().gpioConfigurations_;
    for (const auto& config : configs) {
//筛选
      if (config.second.mode_ == 1) {
        names.push_back(config.second.name_);
      }
    }
  }
  return names;
}
```

###stageDigitalOutputs
```cpp
//设置所有数字输出的状态

//outputs:设置的数字输出状态的布尔向量。
void RmSlaveManager::stageDigitalOutputs(const std::vector<bool>& outputs) {
  auto outputItr = outputs.begin();
//遍历每个从设备
  for (auto& slave : slaves_) {
//获取已暂存的命令
    Command command = slave->getStageCommand();
    auto configs = slave->getConfiguration().gpioConfigurations_;
//每个从设备遍历数字输出配置
    for (const auto& config : configs) {
      if (config.second.mode_ == 1) {
//设置数字输出状态
        command.setDigitalOutput(config.first, *outputItr++);
      }
      if (outputItr == outputs.end()) {
        slave->stageCommand(command);
        return;
      }
    }
  }
}

```