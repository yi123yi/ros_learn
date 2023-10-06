## imu_complementary_filter.cpp

#### 命名空间：
```cpp
   namespace rm_common
```
   - 这里定义了一个名为 `rm_common` 的命名空间，用于封装代码中的类和函数。

#### 类定义：
```cpp
   class ImuComplementaryFilter
```
   - 定义了名为 `ImuComplementaryFilter` 的类。

####  `filterUpdate`：
```cpp
   void ImuComplementaryFilter::filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt)
```
   - 该函数用于更新滤波器。接收 IMU 的加速度（ax, ay, az）、角速度（wx, wy, wz）以及时间间隔 `dt`。

#### `getOrientation`：
```cpp
   void ImuComplementaryFilter::getOrientation(double& q0, double& q1, double& q2, double& q3)
```
   - 该函数用于获取滤波后的姿态四元数。

####  `initFilter`：
```cpp
   bool ImuComplementaryFilter::initFilter(XmlRpc::XmlRpcValue& imu_data)
```
   - 该函数用于初始化滤波器。接收一个 `XmlRpc::XmlRpcValue` 类型的参数 `imu_data`，其中包含了一些 IMU 相关的配置信息。

####   `resetFilter`：
```cpp
   void ImuComplementaryFilter::resetFilter()
```
   - 该函数用于重置滤波器的配置，重新初始化。

####  私有成员变量：
   - `use_mag_`：表示是否使用磁力计信息。
   - `gain_acc_`：加速度信息的增益。
   - `gain_mag_`：磁力计信息的增益。
   - `do_bias_estimation_`：表示是否进行偏置估计。
   - `bias_alpha_`：偏置估计的 alpha 参数。
   - `do_adaptive_gain_`：表示是否使用自适应增益。
   - `filter_`：用于实际的滤波操作。

