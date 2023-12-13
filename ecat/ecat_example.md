##Ethercat

####1.初始化：
   - `rm_ecat::RmSlaveManager` 被实例化为 `rmSlaveManager`，具有特定的配置参数（`true, true, 0.001`）。
   - 创建了一个 `rm_ecat::RmBusManager` 的共享指针 `rmBusManager`。
   - 期望通过命令行参数传递 `setup.yaml` 文件的路径，并用它来配置 `rmBusManager`。

####2.加载配置：
   - `rmBusManager` 使用 `fromFile` 方法从指定的YAML文件（`setup.yaml`）加载配置。

####3. 从设备管理器设置：
   - `rmSlaveManager` 使用加载的 `rmBusManager` 进行配置。
   - 调用 `startup` 方法启动与EtherCAT从设备的通信。

####4. 控制循环：
   - 使用 `any_worker::Worker` 类实现了一个控制循环。
   - controlUpdate 函数被设置为控制循环的回调。该函数从EtherCAT从设备获取速度读数，计算目标速度命令，并将该命令设置为执行。
   - 控制循环以指定的时间步长运行，并以优先级98启动。

#### 5. 主循环:
   - 主循环在 `rmSlaveManager` 运行时等待（可能正在处理EtherCAT通信）。
   - 一旦主循环退出，控制循环 (`controlWorker`) 就会停止。

#### 6. 关闭：
   - 负责EtherCAT通信和控制循环的启动和关闭。


```cpp
#include <rm_ecat_manager/RmSlaveManager.h>

rm_ecat::RmSlaveManager rmSlaveManager(true, true, 0.001);
std::shared_ptr<rm_ecat::RmBusManager> rmBusManager = std::make_shared<rm_ecat::RmBusManager>();

bool controlUpdate(const any_worker::WorkerEvent& /*event*/) {
  auto slaves = rmSlaveManager.getSlaves();
  rm_ecat::Command command;
-
  rm_ecat::Reading reading = slaves[0]->getReading();
  command.setTargetCommand(rm_ecat::CanBus::CAN0, 2, 0.0006 * (0. - reading.getVelocity(rm_ecat::CanBus::CAN0, 2)));

  slaves[0]->stageCommand(command);
  return true;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
    return EXIT_FAILURE;
  }

  rmBusManager->fromFile(argv[1], false, &rmSlaveManager);
  rmSlaveManager.setBusManager(rmBusManager);
  rmSlaveManager.startup();
  std::cout << "Startup finished" << std::endl;

  any_worker::Worker controlWorker("ControlWorker", rmSlaveManager.getTimeStep(), controlUpdate);
  controlWorker.start(98);

  while (rmSlaveManager.isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  controlWorker.stop();
}
```

