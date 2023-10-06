#### **加载参数文件：**
```xml
   <rosparam file="$(find rm_hw)/config/actuator_coefficient.yaml" command="load" ns="rm_hw"/>
   <rosparam file="$(find rm_chassis_controllers)/test/config/test_standard_chassis.yaml" command="load"/>
   <rosparam file="$(find rm_chassis_controllers)/test/urdf/test_standard_chassis.xacro" command="load" ns="robot_description"/>
   <rosparam file="$(find rm_fsm)/test/config/test_power_limit.yaml" command="load"/>
```

 加载了参数文件，包括 `actuator_coefficient.yaml`、`test_standard_chassis.yaml` 和 `test_power_limit.yaml`。

#### ** TF 变换：**
```xml
   <node pkg="tf" type="static_transform_publisher" name="" args="0 0 0 0 0 0 1 world odom 500"/>
```

   将 `world` 坐标系到 `odom` 坐标系的变换

#### **加载控制器：**

```xml
   <node name="controller_spawner" pkg="controller_manager" type="spawner"
        respawn="false"
        output="screen" args="
      rm_hw/robot_state_controller
      rm_hw/joint_state_controller
      rm_hw/chassis_standard_controller
"/>
```

#### **启动节点：**
```xml
   <node name="rm_hw" pkg="rm_hw" type="rm_hw" respawn="false"/>
   <node name="dbus_node" pkg="rm_hw" type="dbus_node" respawn="false"/>
   <node name="rm_fsm" pkg="rm_fsm" type="rm_fsm" respawn="false"/>
```
