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
          ├── /joycon_input/gripper    夹爪状态
          └── /joycon_input/reset      复位请求
          │
          ▼
  teleop_controller
          │
          ├── 相对位姿映射
          ├── 工作空间约束
          ├── URDF 正运动学和 QP/DLS IK
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
| 转动手柄 | 控制末端 roll、pitch、yaw |
| 摇杆前后 | 在水平面内沿手柄 yaw 指向前后移动 |
| 摇杆左右 | 在水平面内沿手柄 yaw 横向移动 |
| 按下摇杆 | 末端下降 |
| `R` / `L` | 末端上升 |
| `ZR` / `ZL` | 切换夹爪打开/关闭 |
| `+` | 重新执行 Joy-Con 静止标定 |
| `Home` | 平滑返回启动时的关节位置；左手柄回退控制时使用 `Capture` |

Joy-Con 启动标定时应水平静置约两秒。姿态由六轴 Mahony 四元数滤波器计算：
陀螺仪负责连续旋转积分，加速度计的重力方向修正 roll/pitch。Joy-Con 没有磁力计，
因此 yaw 没有绝对航向参考，仍可能随陀螺零偏缓慢漂移；按 `+` 重新静止标定可清零。
滤波器直接输出真实弧度，不再使用原实现中旋转后 X 轴的单个分量近似 yaw，也不再做
额外的 `π/1.5` 非线性放大。

滤波参数如下。`attitude_filter_kp` 越大，roll/pitch 回到重力参考的速度越快；
`attitude_filter_ki` 用于缓慢补偿横滚/俯仰陀螺零偏。快速平移时加速度模长偏离 1 g，
超过 `attitude_accel_rejection` 的样本不会被误当成倾斜：

```yaml
attitude_filter_kp: 2.5
attitude_filter_ki: 0.05
attitude_accel_rejection: 0.25
planar_stick_translation: true
```

`planar_stick_translation` 默认开启：摇杆平移方向只随 yaw 旋转，手柄的
pitch/roll 不会再让摇杆命令混入 Z 分量。世界 Z 方向只由 `R`/`L`
上升键和摇杆按下降低键控制。关闭该参数可恢复 vendor 库原有的三维指向平移。

实际按键及摇杆位移的生成逻辑来自内置的 `joyconrobotics` 库。

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

## 3. 自动接合与相对位姿控制原理

控制器在手柄输入与机械臂关节反馈均就绪后自动记录：

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

- 自动接合时，机械臂不会突然跳到 Joy-Con 的绝对位置。
- 遥操持续生效，转动手柄或操作摇杆会立即改变末端目标。
- Home 复位完成后会以当前手柄姿态和复位后的机械臂位姿重新建立映射。

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
orientation_limit: [1.20, 0.20, 0.75]
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
`orientation_limit` 以弧度限制接合后末端相对姿态的旋转向量分量，当前分别约为
Roll ±120°、Pitch ±13°、Yaw ±49°，用于减少手柄姿态进入 IK 不可达区域的概率。

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

### QP + IK（默认）

默认的 `ik_solver: placo` 使用 PlaCo 构造速度级增量 QP。设计参考
[XRoboToolkit Teleop Sample](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)，
并针对 B601-RS 这台非冗余六轴机械臂做了以下适配：

- 固定浮动基座，禁止 QP 通过移动整台机器人“完成”末端任务；
- 把 `joint_margin` 后的软限位直接写入 PlaCo 模型，作为 QP 硬约束；
- 用 `max_joint_speed` 覆盖 URDF 中偏大的模型速度值，作为 QP 硬约束；
- 每个控制周期只推进一个增量 IK 周期，避免重复求解放大允许的运动量；
- 位置和姿态使用独立权重；默认位置权重远高于姿态，转腕时优先锁住 TCP 位置；
- 保留低层发布前的限位检查，拒绝非有限、越界或求解异常的结果。

QP 的简化形式为：

```text
min Δq  wp ||Jp Δq - ep||² + wr ||Jr Δq - er||² + ε ||Δq||²

subject to
  qsoft_min ≤ q + Δq ≤ qsoft_max
  |Δqi| ≤ max_joint_speed × dt
```

末端位姿任务是软任务，因此目标暂时不可达时，求解器仍会返回满足关节和速度约束的
最佳增量，机械臂会移动到最接近的可行姿态，而不是因为未达到误差阈值立即冻结。
`ik_position_tolerance` 和 `ik_orientation_tolerance` 用于判断目标是否到达和记录诊断，
不用于放行不安全结果。

```yaml
ik_position_weight: 100.0
ik_orientation_weight: 0.35
ik_manipulability_weight: 0.0
ik_qp_substeps: 1
```

默认 `100:0.35` 的位置/姿态权重比是有意设置的：当大姿态指令与单周期关节速度约束
冲突时，QP 会先把速度预算用于保持末端位置，再以剩余能力追踪姿态。Home 位姿下施加
49° 的突发 yaw 数值压力测试中，原 `1:0.35` 权重的最大位置漂移约 17.8 cm，当前权重
约 0.9 cm。姿态到达速度会相应降低，这是防止末端大幅扫动所需的安全取舍。

`ik_qp_substeps` 可用于一次控制周期内多次重新线性化。设置为 `N` 时，每个子步使用
`dt/N`，所以总速度边界不会放大。B601-RS 是 6 自由度机械臂，完整 6D 末端任务没有
可供次级目标使用的零空间，因此 manipulability 默认关闭；盲目采用参考项目中的
`1e-2` 权重会与末端位姿竞争并产生漂移。

### DLS 备用后端

把 `ik_solver` 改为 `dls` 可使用内置阻尼最小二乘后端。它通过有限差分雅可比求解：

```text
Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ error
```

其中 `λ` 是 `ik_damping`，用于降低奇异点附近的数值不稳定。上一周期关节命令会作为
下一次求解的初值，使解保持连续。`ik_max_iterations` 只对 DLS 生效。

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
workspace_max: [0.70, 0.55, 0.70]
```

### 关节软限位

控制器使用 URDF 的关节上下限，并通过 `joint_margin` 在机械限位前预留余量：

```yaml
joint_margin: 0.02
```

控制器使用固定的安全 Home 关节姿态，避免从 joint2/joint3 的机械下限直接开始姿态 IK：

```yaml
home_joint_positions: [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
move_home_on_startup: true
```

收到第一帧完整关节反馈后，机械臂会按 `max_joint_speed` 平滑移动到 Home；到达后才建立
手柄与末端的相对位姿锚点。`Home/Capture` 按键也返回同一姿态。

### 关节速度限制

PlaCo QP 内部和发布前安全层都会限制每周期最大关节变化：

```text
最大单步变化 = max_joint_speed × 控制周期
```

默认控制频率为 60 Hz、最大关节速度为 0.7 rad/s，因此单周期最大变化约为
0.0117 rad。

### 输入超时

```yaml
input_timeout: 0.30
```

超过 0.3 秒没有收到手柄位姿时，控制器停止更新 IK 目标并保持上一条合法关节命令。

### IK 失败保护

PlaCo 的末端任务为软任务：不可达目标会被渐进逼近到最近可行姿态。只有 QP 抛出异常、
返回非法数值、越过软关节限位或越过速度边界时，本次结果才会被拒绝，机械臂保持上一条
合法命令。数据日志中的 `tracking_limited` 表示目标尚未达到且至少一个关节已到软限位。

## 8. 夹爪控制

`ZR` 每次按下会切换夹爪状态；只有左手柄参与控制时使用 `ZL`。输入节点发布归一化状态：

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

## 9. 遥操数据日志

控制器默认以控制频率将遥操数据写入 CSV。每次启动会在 `teleop_logs/` 下创建一个带时间戳的
独立文件，例如 `teleop_20260722_143000.csv`。日志包含：

- Joy-Con 输入位置和 roll/pitch/yaw；
- 映射后的末端目标位置和姿态；
- `/joint_states` 的 joint1～joint6 实际反馈；
- 本周期 IK 求出的 joint1～joint6；
- 限速后实际发布的 joint1～joint6 命令及单周期变化量；
- IK/QP 有效性、目标是否到达、位置/姿态残差、限位状态、控制状态、输入延迟和夹爪命令。

相关参数：

```yaml
data_logging: true
data_log_directory: teleop_logs
data_log_flush_interval: 1.0
```

`data_log_directory` 为相对路径时，相对于启动命令的当前目录。若不需要记录，可将
`data_logging` 设置为 `false`。程序每隔 `data_log_flush_interval` 秒刷新文件，并在正常退出时关闭文件。

## 10. 构建

在 JoyReBot 工作区根目录运行：

```bash
cd /home/verser/CPP/JoyReBot
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select joyrebot_teleop
source install/setup.bash
```

Python 运行依赖包括 NumPy、SciPy、`hidapi` 和 `PyGLM`。使用默认 QP 后端还需要：

```bash
python3 -m pip install \
  'scipy>=1.15.3,<1.16' \
  'placo>=0.9.23,<0.10'
```

PlaCo 0.9.23 会安装 NumPy 2.x；Ubuntu 22.04 自带的 SciPy 1.8 与 NumPy 2.x
二进制不兼容，因此必须同时安装上述新版 SciPy。安装时出现其他应用缺少
`huggingface-hub`、`transformers` 或 `tqdm` 的提示与本节点无关，不需要为了遥操作
额外安装这些模型工具。

如果运行环境不能安装 PlaCo，可把 `ik_solver` 改为 `dls`。Joy-Con 的 Linux
内核驱动、蓝牙配对和 udev 权限属于操作系统配置，无法通过 Python 源码内置。

## 11. 使用真实 Joy-Con

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
3. 遥操会在手柄输入和关节反馈就绪后自动接合。
4. 从小幅度动作开始测试各个位置和姿态方向。
5. 如果方向不符合直觉，停止运行并修改 `teleop.yaml`。

## 12. 主要文件

```text
joyrebot_teleop/
├── config/teleop.yaml                 遥操参数
├── config/rebot_b601_kinematics.urdf  独立的运动学模型
├── launch/teleop.launch.py            单独启动遥操节点
├── joyrebot_teleop/
│   ├── joycon_input_node.py            Joy-Con ROS 输入适配
│   ├── teleop_controller.py            IK、安全和关节命令节点
│   ├── placo_solver.py                 受限增量 QP IK 后端
│   ├── pose_mapping.py                 相对位姿及坐标映射
│   ├── kinematics.py                   URDF 正运动学与数值 IK
│   └── vendor/joyconrobotics/
│       ├── attitude.py                 Mahony 四元数姿态滤波
│       └── ...                         内置 Joy-Con 运行时库
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
7. 在仿真中验证不可达目标、输入超时和手柄断连行为。

在所有方向、限位和失效保护验证完成前，不应直接连接真实机械臂。
