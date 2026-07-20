<h1 align="center">JoyReBot</h1>

面向 **reBot B601-RS 六轴机械臂**的游戏手柄直观遥操作方案。通过手柄实现多种可靠的机械臂控制方式，同时采集操作数据，为动作复现、技能学习与遥操优化提供数据支撑。

![reBot B601-RS](assets/view.png)

## 设计目标

- **直觉操控** — 用手柄摇杆和按键直接映射机械臂关节与末端动作，降低示教门槛
- **多模式切换** — 支持关节空间控制、笛卡尔空间控制、夹爪控制等模式，一键切换
- **操作即数据** — 手柄每一次操作自动记录，形成（时间戳、关节角/末端位姿、手柄输入）三元组，无需额外标注流程
- **数据闭环** — 采集的数据可用于动作回放复现、模仿学习训练、遥操作策略评估与优化

## 环境要求

| 组件     | 版本/说明                                                          |
| -------- | ----------------------------------------------------------------- |
| 操作系统 | Ubuntu 22.04                                                      |
| ROS 2    | Humble                                                            |
| 仿真器   | Gazebo Harmonic (`gz-sim 8`)            |
| 构建工具 | colcon                                                            |

### 安装 Gazebo Harmonic 接口

ROS 2 Humble 官方源仅提供 Fortress 接口，需安装 Gazebo 官方包：

```bash
sudo apt-get install -y gz-harmonic ros-humble-ros-gzharmonic
```

## 快速开始

### 1. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动仿真环境

```bash
ros2 launch joyrebot_gazebo_sim sim.launch.py
```

无头模式（服务器/SSH）：

```bash
ros2 launch joyrebot_gazebo_sim sim.launch.py gui:=false
```

### 3. 关节控制接口

仿真启动后，通过 ROS 2 话题直接控制关节：

```bash
# 控制单个关节（单位：rad），关节名 joint1 ~ joint6
ros2 topic pub --once /rebot/joint1/cmd_pos std_msgs/msg/Float64 '{data: 0.5}'

# 控制夹爪（单位：m，范围 0 ~ 0.05）
ros2 topic pub --once /rebot/gripper/cmd_pos std_msgs/msg/Float64 '{data: 0.03}'
```

关节状态通过 `/joint_states` 话题从 Gazebo 桥接到 ROS 2，可用于 `robot_state_publisher` 和数字孪生同步。

### 4. 手柄遥操

> 手柄遥操节点开发中。完成后通过 `ros2 launch` 启动对应控制模式即可。

## 控制模式（规划）

| 模式       | 描述                                                     | 适用场景             |
| ---------- | -------------------------------------------------------- | -------------------- |
| 关节点动   | 手柄按键/摇杆逐一控制六个关节正反转                      | 精细调姿、单轴标定   |
| 末端笛卡尔 | 摇杆控制末端 x/y/z 平移与姿态调整，IK 实时解算           | 抓取放置、轨迹跟踪   |
| 记录回放   | 回放已录制的操作轨迹，关节/末端均支持                    | 复现演示、一致性验证 |
| 数据采集   | 后台自动记录每次操作，支持分段标记和元数据注释           | 数据集构建           |

## 数据格式

每次操作会话生成一条记录，包含：

```json
{
  "session_id": "20240720-001",
  "mode": "joint_jog",
  "label": "抓取方块-右手",
  "records": [
    {
      "timestamp": 0.000,
      "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "gripper_position": 0.0,
      "joystick_input": { "axis_left_x": 0.1, "axis_left_y": -0.2, ... },
      "button_state": { "A": false, "B": false, ... }
    },
    ...
  ]
}
```
