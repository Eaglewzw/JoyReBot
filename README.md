<h1 align="center">JoyReBot</h1>

## 项目简介

面向 **reBot B601-RS 六轴机械臂**的 Joy-Con 遥操作方案。项目提供末端笛卡尔空间与关节空间两种控制方式，并记录操作数据，为动作复现、技能学习与遥操优化提供数据支撑。主要功能包括：

- Gazebo Harmonic 仿真环境与 ROS–Gazebo 话题桥接；
- **末端遥操**：Joy-Con 控制末端相对位姿和夹爪，经 URDF 运动学与数值 IK 输出关节位置命令；
- **关节遥操**：Joy-Con 的 IMU、摇杆和按键直接同时控制六个关节，无需 IK；
- 工作空间、关节限位、速度限制、输入超时和 IK 失败保护；
- 终端状态面板、CSV 遥操数据日志与关节接口自检。

<p align="center">
  <img src="assets/view.png" alt="reBot B601-RS" width="720">
</p>

## 环境要求

| 组件 | 版本/说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 仿真器 | Gazebo Harmonic（`gz-sim 8`） |
| 构建工具 | colcon |


## 快速开始

### 1. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动仿真

```bash
ros2 launch joyrebot_gazebo_sim sim.launch.py
```

无头模式（服务器或 SSH）：

```bash
ros2 launch joyrebot_gazebo_sim sim.launch.py gui:=false
```

### 3. 手柄遥操

#### 自检测试

```bash
ros2 run joyrebot_gazebo_sim joint_self_check
```

#### 末端遥操（IK）

```bash
# Joy-Con 控制末端位姿；请勿与关节遥操同时启动
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch joyrebot_teleop teleop.launch.py
```

#### 关节遥操

```bash
# Joy-Con 直接控制关节；请勿与 IK 遥操同时启动
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch joyrebot_joint_teleop joint_teleop.launch.py
```

两种模式均发布 `/rebot/joint1/cmd_pos` 至 `/rebot/joint6/cmd_pos` 和 `/rebot/gripper/cmd_pos`；关节单位为 rad，夹爪单位为 m。首次使用时请先在仿真中以小幅动作检查方向和限位。

## Joy-Con 操作

右手柄优先；未连接右手柄时可使用左手柄。启动时请将 Joy-Con 水平静置约两秒完成 IMU 标定。两种模式的按键含义不同，请按所启动的模式操作。

### 末端遥操（笛卡尔空间 / IK）

| 操作 | 功能 |
| --- | --- |
| 转动手柄 | 控制末端姿态 |
| 摇杆 | 控制末端水平 XY 平移（方向随手柄 yaw，忽略 pitch/roll） |
| 按下摇杆 | 控制末端下降 |
| `R` / `L` | 控制末端上升 |
| `ZR` / `ZL` | 切换夹爪开关状态 |
| `+`（右）/ `-`（左） | 重新标定手柄 |
| `Home`（右）/ `Capture`（左） | 平滑返回启动时的关节位置 |


### 关节遥操（关节空间）

| 操作 | 功能 |
| --- | --- |
| 手柄 `roll` | 一对一控制 `joint6` 末端旋转 |
| 手柄 `pitch` | 一对一控制 `joint4` 腕俯仰 |
| 手柄 `yaw` | 速度控制 `joint1` 底座回转 |
| 摇杆前后 | 速度控制 `joint2` 大臂 |
| 摇杆左右 | 速度控制 `joint5` 腕滚转 |
| `R` / `L` 与摇杆按下 | 分别正向/反向速度控制 `joint3` 小臂 |
| `ZR` / `ZL` | 切换夹爪开关状态 |
| `B`（右）/ `↓`（左，按住） | 离合：冻结关节，松开后重新锚定 |
| `X`（右）/ `↑`（左） | 以当前手腕姿态重新锚定 |
| `Home`（右）/ `Capture`（左） | 平滑返回 Home 位姿 |
| `+`（右）/ `-`（左） | 重新标定 IMU |

