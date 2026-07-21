# joyrebot_teleop

`joyrebot_teleop` 是 reBot B601-RS 六轴机械臂的 ROS 2 笛卡尔空间遥操作包。
操作者手持右侧 Joy-Con，通过手柄姿态、摇杆和按键控制机械臂末端位姿及夹爪。

Joy-Con 的运行时解析代码已经内置在
`joyrebot_teleop/vendor/joyconrobotics`，使用本项目时不需要另外克隆
`joycon-robotics` 仓库。第三方代码来源和许可证见
`THIRD_PARTY_NOTICES.md`。

本包只负责遥操作，不启动也不依赖 Gazebo。它通过 `/joint_states` 和
`/rebot/*/cmd_pos` ROS 话题连接任意兼容的机械臂控制后端。仿真启动和关节测试由其他
package 独立负责。

## 1. 控制方法概览

本系统不把 Joy-Con 的姿态角直接映射到某几个机械臂关节，而是让 Joy-Con
控制机械臂末端相对于当前位置的六维位姿变化：

```text
Joy-Con IMU、摇杆、按键
          │
          ▼
  joycon_input_node
          │
          ├── /joycon_input/pose       手柄六维位姿
          ├── /joycon_input/clutch     离合/运动使能
          └── /joycon_input/gripper    夹爪状态
          │
          ▼
  teleop_controller
          │
          ├── 相对位姿映射
          ├── 工作空间约束
          ├── URDF 正运动学和数值 IK
          ├── 关节限位和速度限制
          └── 超时及 IK 失败保护
          │
          ▼
 /rebot/joint1/cmd_pos ... /rebot/joint6/cmd_pos
 /rebot/gripper/cmd_pos
          │
          ▼
 机械臂关节位置控制接口
```

因此当前控制类型是：

> Joy-Con 相对位姿输入 → 笛卡尔末端目标 → 逆运动学 → 关节位置控制。

它不是关节直接映射、关节速度控制或力控制。

## 2. 默认操作方式

| Joy-Con 操作 | 功能 |
|---|---|
| 按住 `ZR` | 接合离合，允许机械臂跟随；左手柄回退控制时使用 `ZL` |
| 松开 `ZR` | 停止更新目标，机械臂保持当前位置 |
| 转动手柄 | 控制末端 roll、pitch、yaw |
| 摇杆前后 | 控制末端沿手柄指向前后移动 |
| 摇杆左右 | 控制末端横向移动 |
| 按下摇杆 | 末端下降 |
| `R` | 切换夹爪打开/关闭；左手柄回退控制时使用 `L` |
| `+` | 重新执行 Joy-Con 静止标定 |

Joy-Con 启动标定时应水平静置约两秒。实际按键及摇杆位移的生成逻辑来自内置的
`joyconrobotics` 库。

输入节点默认以 30 Hz 在终端打印左右手柄状态面板，内容包括当前按键、左右摇杆原始值、
roll/pitch/yaw、控制位置及电池状态。当前未连接的一侧显示“未连接”。可以通过以下参数
调整或关闭输出：

```yaml
terminal_display: true
display_rate: 30.0
rescan_rate: 2.0
```

控制优先级固定为右手柄高于左手柄。节点始终分别探测左右手柄：两个都未连接时仍会
运行并显示双侧“未连接”；只连接一侧时自动使用该侧；两侧都连接时使用右手柄控制并
同时显示两侧数据。先连接左手柄、随后连接右手柄时，主控会自动切换到右手柄。
`rescan_rate` 控制未连接手柄的重新探测频率。

## 3. 离合与相对位姿控制原理

`ZR` 是持续按压式的运动使能键，也称为离合键。只有左手柄参与控制时使用对应的
`ZL` 键。按下离合的瞬间，控制器记录：

```text
input_anchor = Joy-Con 当前位姿
robot_anchor = 机械臂当前末端位姿
```

随后只使用 Joy-Con 相对于 `input_anchor` 的变化量。

位置目标为：

```text
Joy-Con 相对位移 = Joy-Con 当前位置 - input_anchor.position
机械臂目标位置 = robot_anchor.position + 映射后的相对位移
```

姿态目标为：

```text
Joy-Con 相对旋转 = 当前旋转 × 基准旋转的逆
机械臂目标旋转 = 映射后的相对旋转 × robot_anchor.rotation
```

这种方法具有以下特点：

- 按下离合时，机械臂不会突然跳到 Joy-Con 的绝对位置。
- 松开离合后，机械臂保持不动，操作者可以调整手臂到更舒适的位置。
- 再次按下离合时，以新的手柄姿态和当前机械臂位姿重新建立映射。
- 操作方式类似抬起鼠标后重新放置，不受操作者手臂活动范围限制。

## 4. 坐标轴映射

Joy-Con 坐标系和机械臂基坐标系可能方向不同。控制器对位置和旋转分别支持：

- 坐标轴交换；
- 坐标方向反转；
- 灵敏度缩放。

配置位于 `config/teleop.yaml`：

```yaml
position_scale: [1.0, 1.0, 1.0]
position_axis_map: [0, 1, 2]
position_axis_sign: [1.0, 1.0, 1.0]

orientation_scale: [1.0, 1.0, 1.0]
orientation_axis_map: [0, 1, 2]
orientation_axis_sign: [1.0, 1.0, 1.0]
```

例如，机械臂 X 方向与期望相反时：

```yaml
position_axis_sign: [-1.0, 1.0, 1.0]
```

需要交换机械臂 X/Y 输入时：

```yaml
position_axis_map: [1, 0, 2]
```

`position_scale` 和 `orientation_scale` 越小，机械臂对手柄运动越不敏感，适合精细操作。

## 5. 运动学和 IK 原理

`kinematics.py` 从本包自带的 `config/rebot_b601_kinematics.urdf` 自动解析以下
串联运动链。该文件只描述运动学，不包含网格、物理参数或仿真插件：

```text
base_link → joint1 → joint2 → joint3 → joint4
          → joint5 → joint6 → gripper_end
```

解析内容包括关节原点、旋转轴、固定变换和关节上下限。

### 正运动学

根据六个关节角逐级相乘齐次变换矩阵，计算 `gripper_end` 在 `base_link`
坐标系中的位置和旋转：

```text
T_end = T1(q1) × T2(q2) × ... × T6(q6) × T_tool
```

### 逆运动学

控制器使用阻尼最小二乘法求解 IK。每次迭代先计算当前末端与目标末端的六维误差：

```text
error = [x, y, z 位置误差, rx, ry, rz 旋转误差]
```

通过有限差分计算数值雅可比矩阵，并使用下式求关节增量：

```text
Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ error
```

其中 `λ` 是阻尼系数，用于降低奇异点附近的数值不稳定。上一周期的关节命令会作为
下一次 IK 的初值，使解保持连续，减少机械臂突然切换到另一组关节解的风险。

## 6. 关节命令

IK 成功后，控制节点向机械臂控制后端发布：

```text
/rebot/joint1/cmd_pos
/rebot/joint2/cmd_pos
/rebot/joint3/cmd_pos
/rebot/joint4/cmd_pos
/rebot/joint5/cmd_pos
/rebot/joint6/cmd_pos
/rebot/gripper/cmd_pos
```

消息类型均为 `std_msgs/msg/Float64`。六个机械臂关节使用弧度，夹爪使用米。

控制器订阅 `/joint_states` 获取当前关节反馈。在收到完整的六关节状态前，不会开始控制。

## 7. 安全机制

### 工作空间限制

末端目标会被限制在配置的三维区域中：

```yaml
workspace_min: [-0.55, -0.55, -0.05]
workspace_max: [0.55, 0.55, 0.70]
```

### 关节软限位

控制器使用 URDF 的关节上下限，并通过 `joint_margin` 在机械限位前预留余量：

```yaml
joint_margin: 0.02
```

### 关节速度限制

每周期允许的最大关节变化为：

```text
最大单步变化 = max_joint_speed × 控制周期
```

默认控制频率为 50 Hz、最大关节速度为 0.7 rad/s，因此单周期最大变化约为
0.014 rad。

### 输入超时

```yaml
input_timeout: 0.30
```

超过 0.3 秒没有收到手柄位姿时，控制器停止更新 IK 目标并保持上一条合法关节命令。

### IK 失败保护

当目标不可达、IK 未收敛或结果包含非法数值时，本次结果不会发送，机械臂保持上一条
合法命令。

## 8. 夹爪控制

`R` 每次按下会切换夹爪状态；只有左手柄参与控制时使用 `L`。输入节点发布归一化状态：

```text
0.0 = 关闭
1.0 = 打开
```

控制节点将其映射为实际行程：

```yaml
gripper_closed: 0.0
gripper_open: 0.05
```

左右夹爪由机械臂控制后端根据同一个夹爪位置命令同步控制。

## 9. 构建

在 JoyReBot 工作区根目录运行：

```bash
cd /home/verser/CPP/JoyReBot
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select joyrebot_teleop
source install/setup.bash
```

Python 运行依赖包括 NumPy、SciPy、`hidapi` 和 `PyGLM`。Joy-Con 的 Linux
内核驱动、蓝牙配对和 udev 权限属于操作系统配置，无法通过 Python 源码内置。

## 10. 使用真实 Joy-Con

确认 Joy-Con 已配对、驱动和设备权限正常，然后运行：

```bash
ros2 launch joyrebot_teleop teleop.launch.py
```

只运行手柄输入和终端数据面板：

```bash
ros2 run joyrebot_teleop joycon_input
```

也支持从源码直接运行（仍需先加载 ROS 2 环境）：

```bash
source /opt/ros/humble/setup.bash
python3 /home/verser/CPP/JoyReBot/src/joyrebot_teleop/joyrebot_teleop/joycon_input_node.py
```

启动后：

1. 将 Joy-Con 水平静置，等待标定完成。
2. 确认机械臂已经稳定并收到 `/joint_states`。
3. 保持 Joy-Con 静止后按住右手柄 `ZR`；使用左手柄回退控制时按住 `ZL`。
4. 从小幅度动作开始测试各个位置和姿态方向。
5. 如果方向不符合直觉，松开 `ZR`/`ZL`，停止运行并修改 `teleop.yaml`。

## 11. 无手柄 Mock 测试

```bash
ros2 launch joyrebot_teleop teleop.launch.py mock:=true
```

Mock 节点会持续接合离合，并生成周期性的 X 方向小幅位移，用于验证：

- ROS 2 节点和话题连接；
- URDF 运动学解析；
- IK 求解；
- 关节命令输出；
- 机械臂控制后端的话题连接。

Mock 模式不代表真实手柄的完整操作行为。

## 12. 主要文件

```text
joyrebot_teleop/
├── config/teleop.yaml                 遥操参数
├── config/rebot_b601_kinematics.urdf  独立的运动学模型
├── launch/teleop.launch.py            单独启动遥操节点
├── joyrebot_teleop/
│   ├── joycon_input_node.py            Joy-Con ROS 输入适配
│   ├── teleop_controller.py            IK、安全和关节命令节点
│   ├── pose_mapping.py                 相对位姿及坐标映射
│   ├── kinematics.py                   URDF 正运动学与数值 IK
│   ├── mock_input_node.py              无手柄测试输入
│   └── vendor/joyconrobotics/          内置 Joy-Con 运行时库
├── test/                               运动学和映射单元测试
└── THIRD_PARTY_NOTICES.md              第三方代码许可证
```

## 13. 调参建议

首次连接真实 Joy-Con 时，建议按以下顺序调整：

1. 在安全的机械臂控制后端中确认 X/Y/Z 三个位置方向。
2. 调整 `position_axis_map` 和 `position_axis_sign`。
3. 将 `position_scale` 暂时设为较小值，例如 `[0.3, 0.3, 0.3]`。
4. 分别确认 roll、pitch、yaw 的方向。
5. 调整 `orientation_axis_map`、`orientation_axis_sign` 和灵敏度。
6. 根据实际任务缩小 `workspace_min/max`。
7. 在仿真中验证不可达目标、松开离合和手柄断连行为。

在所有方向、限位和失效保护验证完成前，不应直接连接真实机械臂。
