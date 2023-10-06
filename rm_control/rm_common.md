# cmakelist


1.项目声明：指定CMake的最低版本和项目名称。
``` c    
    cmake_minimum_required(VERSION 3.10)
    project(rm_common)
```   

2.设置C++标准为C++14。
```c
   set(CMAKE_CXX_STANDARD 14)
   set(CMAKE_CXX_STANDARD_REQUIRED ON)
```
3.添加编译选项
   
      添加编译选项，包括启用所有警告、将警告视为错误以及禁用枚举比较的警告。
```c
   add_definitions(-Wall -Werror -Wno-enum-compare)
```

4.查找依赖包
```c
   find_package(Eigen3 REQUIRED)
   find_package(catkin REQUIRED COMPONENTS
      roscpp
      tf
      rm_msgs
      geometry_msgs
      control_msgs
      controller_manager_msgs
      imu_complementary_filter
      imu_filter_madgwick
      realtime_tools
      dynamic_reconfigure
   )
```
   - 使用`find_package`查找依赖的软件包。

5.构建catkin软件包：
```c
   catkin_package(
      INCLUDE_DIRS
         include
         ${EIGEN3_INCLUDE_DIR}
      CATKIN_DEPENDS
         tf
         rm_msgs
         geometry_msgs
         control_msgs
         controller_manager_msgs
         imu_complementary_filter
         imu_filter_madgwick
         roscpp
         dynamic_reconfigure
      DEPENDS
      LIBRARIES
         rm_common
   )
```

6.包含目录
```c
   include_directories(
      include
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
   )
```
   - 添加头文件包含目录。

7.源文件列表：
```c
   file(GLOB_RECURSE sources "src/*.cpp" "src/decision/*.cpp" "src/filter/*.cpp")
```
   - 使用`file(GLOB_RECURSE ...)`命令获取源文件列表。

8. 构建库和可执行文件：
```c
   add_library(rm_common SHARED ${sources})
   add_executable(test_kalman test/test_kalman_filter.cpp)
```
   - 构建`rm_common`库和`test_kalman`可执行文件。

9. **链接库：**
```cmake
   target_link_libraries(rm_common ${catkin_LIBRARIES})
   target_link_libraries(test_kalman rm_common ${catkin_LIBRARIES})
```
   - 链接依赖库。

10. **安装规则：
```cmake
    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
    )
```
    - 配置安装规则。

11.测试：
```cmake
    if (${CATKIN_ENABLE_TESTING})
        # ...
    endif()
```
    - 配置测试。