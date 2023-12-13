```cpp
namespace rm_ecat {

static bool pathExists(std::string& path) {
  return std::filesystem::exists(path);
}
//    从文件加载EtherCAT总线的配置信息。
//    调用 parseFile 解析配置文件。
//    调用 setup 进行初始化。
bool RmBusManager::fromFile(const std::string& file, bool startup, RmSlaveManager* rmSlaveManager) {
  setupFilepath_ = file;
  parseFile(file);
  setup(startup, rmSlaveManager);
  return true;
}
//    加载并解析配置文件。
//    从配置文件中提取EtherCAT总线和从站的信息。
void RmBusManager::parseFile(std::string path) {
  if (!pathExists(path)) {
    throw std::runtime_error("[RmBusManager] File not found: " + path);
  }
  YAML::Node node = YAML::LoadFile(path);

  if (node["ethercat_master"] != nullptr) {
    const auto masterNode = node["ethercat_master"];
    if (masterNode["time_step"] != nullptr) {
      timeStep_ = masterNode["time_step"].as<double>();
    } else {
      throw std::runtime_error("[RmBusManager] Node time_step missing in ethercat_master");
    }
  } else {
    throw std::runtime_error("[RmBusManager] Node ethercat_master is missing in yaml");
  }

  if (node["ethercat_devices"] != nullptr) {
    // Get all children
    const YAML::Node& nodes = node["ethercat_devices"];
    if (nodes.size() == 0) {
      throw std::runtime_error("[RmBusManager] No devices defined in yaml");
    }

    // Iterate through child nodes
    for (YAML::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
      const YAML::Node& child = *it;
      EthercatSlaveEntry entry;
      // type
      if (child["type"] != nullptr) {
        auto type_str = child["type"].as<std::string>();

        if (type_str == "Rm") {
          entry.type = EthercatSlaveType::Rm;
        } else {
          throw std::runtime_error("[RmBusManager] " + type_str + " is an undefined type of ethercat device");
        }
      } else {
        throw std::runtime_error("[RmBusManager] Node: " + child.Tag() + " has no entry type");
      }

      // name
      if (child["name"] != nullptr) {
        entry.name = child["name"].as<std::string>();
      } else {
        throw std::runtime_error("[RmBusManager] Node: " + child.Tag() + " has no entry name");
      }

      // configuration_file
      if (child["configuration_file"] != nullptr) {
        entry.configFilePath = child["configuration_file"].as<std::string>();
      } else {
        throw std::runtime_error("[RmBusManager] Node: " + child.Tag() + " has no entry configuration_file");
      }

      // ethercat_bus_address
      if (child["ethercat_address"] != nullptr) {
        entry.ethercatAddress = child["ethercat_address"].as<int>();
      } else {
        throw std::runtime_error("[RmBusManager] Node: " + child.Tag() + " has no entry ethercat_bus_address");
      }

      // ethercat_bus
      if (child["ethercat_bus"] != nullptr) {
        entry.ethercatBus = child["ethercat_bus"].as<std::string>();
      } else {
        throw std::runtime_error("[RmBusManager] Node: " + child.Tag() + " has no entry ethercat_bus");
      }
      slaveEntries_.push_back(entry);
    }
  } else {
    throw std::runtime_error("[RmBusManager] Node ethercat_devices missing in yaml");
  }
}
//    配置EtherCAT总线和从站。
//    添加总线和从站到相应的管理器中。
void RmBusManager::setup(bool startup, RmSlaveManager* rmSlaveManager) {
  std::vector<std::unique_ptr<soem_interface::EthercatBusBase>> ethercatBuses;
  for (auto& entry : slaveEntries_) {
    bool busFound = false;
    soem_interface::EthercatBusBase* correspondingBus = nullptr;

    for (auto& bus : ethercatBuses) {
      if (bus->getName() == entry.ethercatBus) {
        busFound = true;
        correspondingBus = bus.get();
        break;
      }
    }
    if (!busFound) {
      ethercatBuses.push_back(std::make_unique<soem_interface::EthercatBusBase>(entry.ethercatBus));
      correspondingBus = ethercatBuses.back().get();
    }

    MELO_DEBUG_STREAM("[RmBusManager] Creating slave: " << entry.name);

    std::shared_ptr<soem_interface::EthercatSlaveBase> slave;
    std::string configurationFilePath = handleFilePath(entry.configFilePath, setupFilepath_);

    switch (entry.type) {
      case EthercatSlaveType::Rm: {
          // 初始化从站，并把从站对象传给rm
          // 并把rm添加进从站管理者里
        auto rm = RmEcatSlave::deviceFromFile(configurationFilePath, entry.name, entry.ethercatAddress);
        rm->setTimeStep(timeStep_);
        slave = rm;
        if (rmSlaveManager != nullptr) {
          if (!rmSlaveManager->addSlave(rm)) {
            throw std::runtime_error("[RmBusManager] could not add slave: " + rm->getName() + " to RmBusManager");
          }
        }
        break;
      }
      default:
        throw std::runtime_error("[RmBusManager] Not existing EthercatSlaveType passed");
    }
      // 由于我们调用的是 soem_interface 里的 EthercatBusManagerBase 里的 readAllBuses() 来读取bus的数据
      // 所以我们也要初始化 EthercatBusBase 里的 bus(实际上是slave)
    if (!correspondingBus->addSlave(slave)) {
      throw std::runtime_error("[RmBusManager] could not add slave: " + slave->getName() +
                               " to bus on interface: " + correspondingBus->getName());
    }
      // 由于我们调用的是 soem_interface 里的 EthercatSlaveManagerBase 里的 sendSdoRead() 来读取bus的数据
      // 所以我们也要初始化 EthercatSlaveBase 里的 bus(真正的bus)
    slave->setEthercatBusBasePointer(correspondingBus);
  }

  if (rmSlaveManager != nullptr) {
    rmSlaveManager->setTimeStep(timeStep_);
  }

  for (auto& bus : ethercatBuses) {
    addEthercatBus(std::move(bus));
  }

  if (startup) {
    startupAllBuses();
  }
}

```
### handleFilePath

```cpp
//const std::string& path: 要处理的文件路径。
// const std::string& setup_file_path: 与路径关联的设置文件的路径。
std::string RmBusManager::handleFilePath(const std::string& path, const std::string& setup_file_path) {
  std::string resultPath;
//处理绝对路径
  if (path.front() == '/') {
    resultPath = path;
    // Path to the configuration file is absolute, we can use it as is.
  } else if (path.front() == '~') {
//   处理带有家目录的路径
    // Path to the configuration file is absolute, we need to replace '~' with the home directory.
    const char* homeDirectory = getenv("HOME");
    if (homeDirectory == nullptr) {
      throw std::runtime_error("[RmBusManager] Environment variable 'HOME' could not be evaluated.");
    }
    resultPath = path;
    resultPath.erase(resultPath.begin());
    resultPath = homeDirectory + resultPath;
  } else {
//处理相对路径
    // Path to the configuration file is relative, we need to append it to the path of the setup file.
    resultPath = setup_file_path.substr(0, setup_file_path.find_last_of('/') + 1) + path;
  }
//检查文件路径是否存在
  if (!pathExists(resultPath)) {
    throw std::runtime_error("Path: " + resultPath + " does not exist");
  }
  return resultPath;
}

}  // namespace rm_ecat

```