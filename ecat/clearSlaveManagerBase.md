```cpp
//管理 EtherCAT 总线上的从设备
namespace cleardrive {

using ClearBusManagerPtr = std::shared_ptr<rm_ecat::RmBusManager>;

template <typename SlaveType>
class ClearSlaveManagerBase {
  using SlavePtr = std::shared_ptr<SlaveType>;
  using Slaves = std::vector<SlavePtr>;

 public:
  // Setup and configuration
  ClearSlaveManagerBase(bool standalone, bool installSignalHandler, double timeStep);
  virtual ~ClearSlaveManagerBase();
  void setTimeStep(double timeStep); //设置时间步长
  double getTimeStep() const { return timeStep_; } //获取时间步长
  void setBusManager(const ClearBusManagerPtr& busManager) { busManager_ = busManager; } //设置总线管理器
  bool addSlave(const SlavePtr& slave); //添加从设备
  bool slaveExists(const std::string& name) const; //检查从设备是否存在
  SlavePtr getSlave(const std::string& name) const;//获取特定名称的从设备。
  Slaves getSlaves() const { return slaves_; } //获取所有从设备
  unsigned int getNumberOfSlaves() const { return slaves_.size(); }//
  std::vector<std::string> getNamesOfSlaves() const;//获取从设备数量

  // Execution
  virtual bool startup();// 启动从设备管理器。
  virtual bool update();//更新从设备管理器
  virtual void updateCommunicationManagerReadMessages();
  virtual void updateProcessReadings();
  virtual void updateSendStagedCommands();
  virtual void updateCommunicationManagerWriteMessages();
  virtual void shutdown();//关闭从设备管理器
  bool isRunning() const { return isRunning_; }//检查管理器的运行状态
  bool shutdownRequested() const { return shutdownRequested_; }//关闭请求状态。

  // Control
  template <typename CommandType>
  void stageCommands(const std::vector<CommandType>& commands);
  template <typename ReadingType>
  std::vector<ReadingType> getReadings() const;

 protected:
  bool updateWorkerCb(const any_worker::WorkerEvent& event);
  void requestShutdown(); //请求关闭从设备管理器
  void handleSignal(int signum);//处理信号的函数

  bool standalone_ = true; //是否是独立运行
  bool installSignalHandler_ = true;//是否需要安装信号处理程序。
  std::shared_ptr<any_worker::Worker> updateWorker_;
  double timeStep_ = 0.0;//时间步长
  std::recursive_mutex isRunningMutex_; //互斥锁，保护 isRunning_ 的访问
  bool isRunning_ = false;
  std::atomic<bool> shutdownRequested_;//是否请求关闭

  ClearBusManagerPtr busManager_; //总线管理器
  Slaves slaves_;//从设备列表
};

}  // namespace cleardrive


```