# ROS2 通用 TF 管理模块设计方案

## 1. 项目目标

设计一个可独立部署、低耦合、易迁移的 ROS2 TF 管理模块，用于统一管理机器人系统中的静态 TF、动态 TF 状态检查、TF 树诊断和配置加载。

项目建议命名：

```text
omni_tf_manager
```

核心目标：

- 不依赖具体机器人型号
- 不依赖具体 SLAM、导航、相机、雷达算法
- ROS2 Humble 可直接使用
- 通过 YAML 配置即可迁移到不同项目
- 尽量不修改业务代码
- 支持静态 TF 集中管理
- 支持 TF 树自动检查
- 支持 TF 冲突和断链检测
- 支持配置热加载
- 后续可扩展 Web UI
- 支持机器人、机械臂、机器狗、AGV 等不同平台

---

# 2. 设计原则

整个模块遵循：

```text
配置驱动
+
TF 发布与 TF 检测分离
+
不绑定具体机器人
+
不绑定 frame 名称
+
不绑定传感器类型
```

不应该在代码中出现：

```cpp
"base_link"
"lidar_link"
"camera_link"
"map"
"odom"
```

这些 frame 名称全部通过配置文件传入。

例如：

```yaml
root_frame: base_link
```

因此切换到另一个机器人时，只需要修改：

```text
config/*.yaml
```

而不需要重新编译。

---

# 3. 系统架构

```text
                omni_tf_manager
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ↓              ↓              ↓
 TF Config Loader   TF Publisher   TF Monitor
        │              │              │
        ↓              ↓              ↓
     YAML         /tf_static       /tf
                                 /tf_static
                                      │
                                      ↓
                                TF Tree Check
                                      │
                   ┌──────────────────┼───────────────┐
                   ↓                  ↓               ↓
                Missing           Conflict          Loop
                 Frame             TF              Detection
```

模块本身不负责 SLAM、定位、导航。

例如：

```text
FastLIO
   │
   └── map → body

Robot Driver
   │
   └── body → base_link

omni_tf_manager
   │
   ├── base_link → lidar
   ├── base_link → camera
   └── base_link → imu
```

各模块之间保持完全解耦。

---

# 4. 推荐目录结构

```text
omni_tf_manager/
│
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── config/
│   ├── tf_manager.yaml
│   └── tf_example.yaml
│
├── launch/
│   └── tf_manager.launch.py
│
├── include/
│   └── omni_tf_manager/
│       ├── config_loader.hpp
│       ├── tf_manager.hpp
│       └── tf_monitor.hpp
│
├── src/
│   ├── config_loader.cpp
│   ├── tf_manager_node.cpp
│   └── tf_monitor_node.cpp
│
└── srv/
    ├── ReloadTF.srv
    ├── CheckTF.srv
    └── ListTF.srv
```

后续如果增加 Web：

```text
web/
├── backend/
└── frontend/
```

Web 部分应作为独立模块，不影响核心 TF Manager。

---

# 5. 模块划分

## 5.1 TF Config Loader

负责读取 YAML。

职责：

```text
读取配置
校验配置
解析 XYZ
解析 RPY
解析 Quaternion
检测重复 child frame
生成内部 TF 数据结构
```

建议内部统一转换成：

```cpp
struct TransformConfig
{
    std::string parent_frame;
    std::string child_frame;

    double x;
    double y;
    double z;

    double qx;
    double qy;
    double qz;
    double qw;

    bool enabled;

    std::string description;
};
```

即：

```text
YAML
 ↓
RPY / Quaternion
 ↓
统一 Quaternion
 ↓
geometry_msgs::msg::TransformStamped
```

---

# 6. YAML 配置设计

建议 TF 配置全部采用独立 YAML。

例如：

```yaml
tf_manager:

  ros__parameters:

    config_version: "1.0"

    publish_static_tf: true

    enable_monitor: true

    transforms:

      base_to_lidar:
        enabled: true

        parent: base_link
        child: lidar_link

        translation:
          x: 0.20
          y: 0.00
          z: 0.35

        rotation_rpy:
          roll: 0.0
          pitch: 0.0
          yaw: 0.0

        description: "Main LiDAR"

      base_to_camera:
        enabled: true

        parent: base_link
        child: camera_link

        translation:
          x: 0.25
          y: 0.00
          z: 0.30

        rotation_rpy:
          roll: 0.0
          pitch: 0.0
          yaw: 0.0

        description: "Front camera"

      base_to_imu:
        enabled: true

        parent: base_link
        child: imu_link

        translation:
          x: 0.0
          y: 0.0
          z: 0.10

        rotation_rpy:
          roll: 0.0
          pitch: 0.0
          yaw: 0.0

        description: "IMU"
```

---

# 7. Quaternion 支持

同时应该支持 Quaternion。

例如：

```yaml
base_to_sensor:

  enabled: true

  parent: base_link
  child: sensor_link

  translation:
    x: 0.2
    y: 0.0
    z: 0.3

  rotation_quaternion:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

规则：

```text
rotation_rpy
rotation_quaternion
```

二选一。

如果同时存在：

```text
ERROR
```

启动失败。

避免产生歧义。

---

# 8. TF Publisher

TF Publisher 只负责：

```text
读取配置
↓
转换 geometry_msgs
↓
StaticTransformBroadcaster
↓
发布 /tf_static
```

建议使用：

```cpp
tf2_ros::StaticTransformBroadcaster
```

不要人为高频发布静态 TF。

错误方式：

```text
100Hz 发布：

base_link → lidar
```

正确方式：

```text
启动时发布一次：

/tf_static
```

---

# 9. TF Monitor

这是项目非常重要的部分。

负责监听：

```text
/tf
/tf_static
```

构建当前机器人 TF 拓扑。

内部维护：

```text
parent → child
```

例如：

```text
map
 ↓
odom
 ↓
base_link
 ├── lidar
 ├── camera
 └── imu
```

内部可以使用：

```cpp
std::unordered_map<
    std::string,
    std::string
>
child_to_parent;
```

以及：

```cpp
std::unordered_map<
    std::string,
    std::vector<std::string>
>
parent_to_children;
```

---

# 10. TF 自动检查功能

建议至少支持以下检查。

---

## 10.1 Multiple Parent

TF 中：

```text
camera_link
```

理论上只能有一个直接 parent。

例如：

```text
base_link → camera_link

body_link → camera_link
```

检测到后输出：

```text
[ERROR]

TF CONFLICT

child:
camera_link

parent 1:
base_link

parent 2:
body_link
```

---

# 11. TF Loop 检测

例如：

```text
A → B
B → C
C → A
```

属于非法 TF Tree。

可以通过 DFS 实现。

例如：

```text
A
↓
B
↓
C
↓
A
```

输出：

```text
[ERROR]

TF LOOP DETECTED

A
→ B
→ C
→ A
```

---

# 12. Required TF 检查

允许用户配置关键 TF。

例如：

```yaml
monitor:

  required_transforms:

    - parent: map
      child: base_link

    - parent: base_link
      child: lidar_link

    - parent: base_link
      child: camera_link
```

系统定时检查。

输出：

```text
TF STATUS

[OK]
map → base_link

[OK]
base_link → lidar_link

[ERROR]
base_link → camera_link
```

---

# 13. Required Frame 检查

也可以只检查 frame 是否存在：

```yaml
required_frames:

  - map
  - odom
  - base_link
  - lidar_link
  - camera_link
  - imu_link
```

如果缺失：

```text
[WARN]

Missing TF Frame:

imu_link
```

---

# 14. TF 更新时间检查

动态 TF 应该监控更新时间。

例如：

```yaml
monitor:

  timeout: 1.0
```

如果：

```text
map → base_link
```

超过 1 秒没有更新：

```text
[ERROR]

TF timeout

map → base_link

Last update:

3.25 sec ago
```

这个对于 SLAM、定位故障非常有用。

---

# 15. TF 频率统计

动态 TF 可以统计：

```text
frequency
latency
last timestamp
```

例如：

```text
TF Monitor

map → base_link
frequency: 50.2 Hz
latency: 12 ms

base_link → camera
STATIC

base_link → lidar
STATIC
```

---

# 16. TF 来源追踪

建议后续增加 TF 发布源检测。

理想效果：

```text
map → base_link

Publisher:

/fastlio_mapping
```

这样以后遇到：

```text
这个 TF 到底谁发的？
```

可以快速定位。

不过需要注意：

ROS2 TF 原始消息本身不会直接包含：

```text
publisher node name
```

因此需要结合：

```text
ROS2 Topic Graph
```

进一步分析。

第一版本可以不实现。

---

# 17. Service 设计

建议提供三个 Service。

---

## 17.1 Reload

```text
/tf_manager/reload
```

用途：

```text
修改 YAML
↓
reload
↓
重新加载
```

例如：

```bash
ros2 service call \
/tf_manager/reload \
std_srvs/srv/Trigger
```

---

# 18. Check

```text
/tf_manager/check
```

手动执行 TF 检测。

例如：

```bash
ros2 service call \
/tf_manager/check \
std_srvs/srv/Trigger
```

返回：

```text
TF Tree OK

Frames: 12
Transforms: 11
Static: 8
Dynamic: 3

No loop detected
No multiple-parent detected
```

---

# 19. List

```text
/tf_manager/list
```

用于列出当前 TF。

例如：

```text
map → odom
odom → base_link
base_link → lidar
base_link → camera
base_link → imu
```

第一版可以直接通过日志输出。

后续再定义自定义 srv。

---

# 20. Topic 输出

建议同时发布：

```text
/tf_manager/status
```

消息可以先使用：

```text
std_msgs/msg/String
```

例如：

```json
{
  "status": "WARNING",
  "frames": 12,
  "transforms": 11,
  "errors": 1
}
```

正式版本推荐定义：

```text
TFManagerStatus.msg
```

例如：

```text
uint8 OK=0
uint8 WARNING=1
uint8 ERROR=2

uint8 status

uint32 frame_count
uint32 transform_count

string[] warnings
string[] errors
```

---

# 21. Launch 设计

建议所有参数外部传入。

```python
tf_manager = Node(
    package="omni_tf_manager",
    executable="tf_manager_node",
    name="tf_manager",
    output="screen",
    parameters=[
        config_file
    ]
)
```

使用：

```bash
ros2 launch omni_tf_manager tf_manager.launch.py \
config:=/path/to/tf.yaml
```

这样同一个二进制程序：

```text
Robot A
Robot B
Robot C
```

只需要加载不同配置。

---

# 22. 推荐的迁移方式

整个模块迁移到其他机器人时：

```text
代码：

不改
```

只新增：

```text
config/
```

例如：

```text
robot_a_tf.yaml

robot_b_tf.yaml

robot_c_tf.yaml
```

启动：

```bash
ros2 launch omni_tf_manager tf_manager.launch.py \
config:=robot_a_tf.yaml
```

或者：

```bash
ros2 launch omni_tf_manager tf_manager.launch.py \
config:=robot_b_tf.yaml
```

---

# 23. 更推荐的工程方式

不要让：

```text
omni_tf_manager
```

保存每一个机器人的配置。

更好的方式是：

```text
omni_tf_manager
```

只保存：

```text
通用代码
```

每个机器人自己的工程保存：

```text
robot_bringup/
└── config/
    └── tf.yaml
```

例如：

```text
omni_tf_manager
   ↑
   │
omni_dog_bringup
   └── config/tf.yaml
```

另外一个项目：

```text
omni_tf_manager
   ↑
   │
agv_bringup
   └── config/tf.yaml
```

这样模块才能真正做到通用。

---

# 24. 推荐项目依赖关系

应该设计成：

```text
                    omni_tf_manager

                    ↑       ↑
                    │       │

         dog_bringup        agv_bringup

              ↑                 ↑

         Dog Project        AGV Project
```

而绝对不要：

```text
omni_tf_manager
   │
   ├── dog
   ├── agv
   ├── arm
   └── drone
```

否则它很快就会变成和业务强绑定的工程。

---

# 25. TF 管理职责边界

这里非常重要。

TF Manager 不应该接管所有 TF。

建议划分为三类。

---

## A. URDF Fixed TF

机器人机械结构：

```text
base_link
 ├── body_link
 ├── lidar_mount
 ├── camera_mount
 └── imu_mount
```

最好由：

```text
URDF
+
robot_state_publisher
```

负责。

---

## B. Configurable TF

例如：

```text
lidar_mount → lidar
camera_mount → camera
```

属于：

```text
传感器安装外参
```

可以由：

```text
omni_tf_manager
```

负责。

---

## C. Dynamic TF

例如：

```text
map → odom

odom → base_link
```

应该由：

```text
SLAM
Localization
Driver
EKF
```

负责。

TF Manager：

```text
只监控
```

不要去发布。

---

# 26. 推荐 TF 架构

一个标准移动机器人建议：

```text
map
 │
 │ Localization / SLAM
 ↓
odom
 │
 │ Driver / EKF
 ↓
base_link
 │
 ├── body_link
 │
 ├── lidar_mount
 │      │
 │      └── lidar_link
 │
 ├── camera_mount
 │      │
 │      └── camera_link
 │
 └── imu_link
```

职责：

```text
map → odom
SLAM

odom → base_link
Driver / EKF

base_link → xxx_mount
URDF

xxx_mount → sensor
TF Manager
```

---

# 27. 为什么保留 mount frame

例如雷达。

不推荐：

```text
base_link → mid360
```

更推荐：

```text
base_link
 ↓
lidar_mount
 ↓
mid360
```

因为：

```text
lidar_mount
```

代表机械安装位。

而：

```text
mid360
```

代表传感器自身 frame。

以后更换：

```text
Mid360
↓
Hesai
```

只需要：

```text
lidar_mount → hesai
```

机器人机械坐标系不需要改变。

这对于跨项目迁移非常有价值。

---

# 28. Namespace 支持

模块必须支持 namespace。

例如：

```text
/robot1

/robot2
```

否则多机器人场景会很麻烦。

启动：

```bash
ros2 launch omni_tf_manager tf_manager.launch.py \
namespace:=robot1
```

但是需要注意：

TF frame 本身通常不会自动加入 ROS namespace。

因此建议提供：

```yaml
frame_prefix: ""
```

例如：

```yaml
frame_prefix: "robot1/"
```

最终：

```text
robot1/base_link

robot1/lidar_link
```

不过单机器人项目：

```text
frame_prefix: ""
```

即可。

---

# 29. 配置 Schema 校验

建议启动时严格检查 YAML。

例如：

```text
missing parent

missing child

parent == child

rotation missing

quaternion invalid

duplicate child

duplicate transform
```

任何严重错误：

```text
直接拒绝启动
```

不要带着错误 TF 继续运行。

例如：

```text
[FATAL]

Invalid TF configuration

Transform:

base_to_camera

Reason:

parent frame equals child frame
```

---

# 30. 日志设计

统一日志格式。

例如：

```text
[TF_MANAGER]

Loaded 8 static transforms

[TF_MANAGER]

Published:

base_link → lidar_link

[TF_MONITOR]

TF tree:

12 frames
11 transforms

[TF_MONITOR][WARNING]

Missing:

camera_link
```

以后排查机器人日志会方便很多。

---

# 31. 第一阶段 MVP

建议第一阶段只实现：

```text
YAML 配置加载

Static TF 发布

RPY / Quaternion

配置校验

TF Tree Monitor

Missing Frame 检测

Multiple Parent 检测

Loop 检测

Required Transform 检测

TF Timeout 检测

Reload Service

Check Service
```

这个版本就已经非常实用了。

不建议第一阶段就做：

```text
Web UI
数据库
ROS Bridge
远程编辑
自动标定
```

这些功能后续逐步增加。

---

# 32. 第二阶段

增加：

```text
TF frequency

TF latency

TF diagnostics

diagnostic_msgs

TF Tree JSON

ROS2 topic status
```

同时接入：

```text
diagnostic_aggregator
```

这样可以统一到 ROS Diagnostic。

---

# 33. 第三阶段 Web TF Manager

再增加：

```text
omni_tf_web
```

架构：

```text
Browser

   ↓ WebSocket

TF Web Server

   ↓ ROS2

omni_tf_manager

   ↓

TF Tree
```

页面：

```text
TF TREE

map
 │
 odom
 │
 base_link
 ├── lidar
 ├── camera
 └── imu
```

点击：

```text
camera
```

显示：

```text
Parent:

base_link

Translation:

X 0.25
Y 0
Z 0.32

Rotation:

Roll   0
Pitch  0
Yaw    0
```

静态 TF 可以：

```text
Edit
Save
Reload
```

动态 TF：

```text
只读
```

---

# 34. 第四阶段：TF 配置持久化

Web 修改以后：

```text
Browser
↓
TF Manager
↓
更新 YAML
↓
Reload
```

这样现场调试传感器安装外参时非常方便。

例如调整：

```text
camera pitch
```

不需要：

```text
改 launch
↓
重新编译
↓
重新启动
```

而是：

```text
修改参数
↓
Apply
```

---

# 35. 与标定模块结合

后续可以对接：

```text
LiDAR Camera Calibration

Hand Eye Calibration

Camera IMU Calibration
```

标定输出：

```text
T_lidar_camera
```

直接写入：

```text
tf.yaml
```

然后：

```text
reload
```

最终形成：

```text
Calibration
    ↓
TF Config
    ↓
TF Manager
    ↓
Robot
```

---

# 36. 推荐参数配置

主配置：

```yaml
tf_manager:

  ros__parameters:

    config_version: "1.0"

    enabled: true

    frame_prefix: ""

    publish_static_tf: true

    enable_monitor: true

    monitor_rate: 1.0

    dynamic_tf_timeout: 1.0

    enable_loop_check: true

    enable_multiple_parent_check: true

    enable_required_frame_check: true
```

---

# 37. 建议区分两个配置文件

更推荐：

```text
tf_manager.yaml
```

负责：

```text
程序行为
```

例如：

```text
检测周期
timeout
功能开关
```

以及：

```text
robot_tf.yaml
```

负责：

```text
机器人 TF
```

例如：

```text
base → lidar
base → camera
```

这样更加清晰。

---

# 38. 最终推荐目录

通用库：

```text
omni_tf_manager/
├── include/
├── src/
├── launch/
├── srv/
├── msg/
├── config/
│   └── default.yaml
└── README.md
```

项目：

```text
omni_dog/
└── omni_dog_bringup/
    ├── launch/
    └── config/
        └── tf.yaml
```

AGV：

```text
omni_agv/
└── omni_agv_bringup/
    ├── launch/
    └── config/
        └── tf.yaml
```

两者共享：

```text
omni_tf_manager
```

---

# 39. 对外接口建议

最终保证整个组件只有几个稳定接口：

```text
Input

YAML Config

/tf

/tf_static
```

输出：

```text
/tf_static

/tf_manager/status
```

Service：

```text
/tf_manager/reload

/tf_manager/check

/tf_manager/list
```

以后内部代码如何修改，都不影响其他项目。

---

# 40. 推荐最终架构

```text
                        Robot Project

        ┌────────────────────┴────────────────────┐

        │                                         │

   robot_bringup                           SLAM / Driver
        │                                         │
        │ YAML                                    │
        ↓                                         ↓

               omni_tf_manager
                     │
        ┌────────────┼────────────┐
        ↓            ↓            ↓

 Config Loader   TF Publisher   TF Monitor

        │            │            │
        │            ↓            ↓

        │        /tf_static      /tf

        │                         │
        └──────────────┬──────────┘
                       ↓

                     TF Tree
                       │
                       ↓

                 Diagnostics
```

---

# 41. 最终开发建议

第一版建议不要把项目做得太重。

核心代码甚至可以控制在：

```text
2000～4000 行 C++
```

核心只解决四个问题：

```text
1. 谁负责发布静态 TF

2. 当前 TF 树是不是完整

3. 当前 TF 树有没有冲突

4. 当前关键 TF 是否正常
```

这样它才容易成为真正可以长期复用的基础组件。

整个模块应该做到：

```text
换机器狗
不用改代码

换 AGV
不用改代码

换机械臂
不用改代码

换 LiDAR
不用改代码

换 Camera
不用改代码

换 SLAM
不用改代码
```

原则就是：

> **TF Manager 只理解 TF，不理解机器人业务。**

只要保持这一点，这个项目就能够长期作为机器人软件基础设施复用。