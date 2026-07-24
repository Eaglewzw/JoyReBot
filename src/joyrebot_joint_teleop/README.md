# joyrebot_joint_teleop

reBot B601-RS 的**拟人关节控制**包。不经过 IK,手腕和摇杆直接驱动关节,**六个关节同时可控**。

> **与 `joyrebot_teleop` 互斥,每次只能运行一个。**
> 两者都独占同一个 Joy-Con HID 设备,并且发布同一组 `/rebot/joint*/cmd_pos` 话题。
> 节点启动 2 秒后会检查命令话题上的其他发布者,发现冲突打 `ERROR`。

## 通道映射

Joy-Con 恰好提供六个独立通道——三个 IMU 旋转、两个摇杆轴、一对按键——正好对应六个关节,**没有任何模式切换**。

| Joy-Con 输入 | 关节 | 人手类比 | 语义 |
| --- | --- | --- | --- |
| 手柄 **roll**(手腕侧转) | `joint6` 末端旋转 | 转手掌 | **绝对** |
| 手柄 **pitch**(手腕上下翻) | `joint4` 腕俯仰 | 抬/压手腕 | **绝对** |
| 手柄 **yaw**(手腕左右摆) | `joint1` 底座回转 | 转身 | 速度 |
| 摇杆 **垂直** | `joint2` 大臂 | 抬大臂 | 速度 |
| 摇杆 **水平** | `joint5` 腕滚转 | 侧摆手腕 | 速度 |
| `R` / 摇杆按下 | `joint3` 小臂 | 伸/曲肘 | 速度 |
| `ZR` | 夹爪 | 抓握 | 切换 |

`R` 为正方向,摇杆按下为负方向。左手柄对应 `L` / 摇杆按下 / `ZL`。

### 为什么 roll/pitch 是绝对而 yaw 是速度

IMU 的 roll 和 pitch 有重力作为绝对基准,不漂移——这两个自由度做成一比一跟随最像人手,手腕怎么摆关节就怎么摆。yaw 没有任何绝对参考、必然漂移;做成绝对映射会导致底座自己慢慢转。所以 yaw 默认给速度语义(手腕偏离锚点多少 → 底座转多快),配合死区完全消除漂移。

想要 yaw 也完全跟随手腕:把 `channel_mode` 第三项改成 `absolute`、`channel_deadzone` 第三项改成 `0.0`,操作中用 `X` 键随时重新锚定。

### 离合 `B`(按住)—— 绝对映射的必需品

你手腕活动范围只有 ±90° 左右,关节行程比这大得多。按住 `B` 冻结全部关节,把手腕摆回舒服的位置,松开继续——就像鼠标推到桌边要抬起来。松开时自动重新锚定,**不会跳变**。

### 其余按键

| 键(右 / 左) | 作用 |
| --- | --- |
| `B` / `↓`(按住) | **离合**,冻结全部关节 |
| `X` / `↑` | 重新锚定到当前手腕姿态 |
| `Home` / `Capture` | 平滑回 Home |
| `+` / `-` | IMU 重标定(vendor 内建,约 2 秒,期间手柄需静置) |

`Y`、`A`(左手柄 `←`、`→`)未绑定。

## 与末端遥操的分工

| | `joyrebot_teleop`(末端) | 本包(关节) |
| --- | --- | --- |
| 控制量 | 末端位姿(IK 求解) | 关节角(直接映射/积分) |
| 适用 | 抓取、对准 | 拟人整臂操作、脱困、摆位 |
| IK 失败 | 会发生(实测 1.5%) | 不存在 |

## 快速开始

```bash
colcon build --symlink-install --packages-select joyrebot_teleop joyrebot_joint_teleop
source install/setup.bash

# 终端 1:仿真
ros2 launch joyrebot_gazebo_sim sim.launch.py

# 终端 2:关节控制 —— 不要同时运行 joyrebot_teleop 的 teleop.launch.py
ros2 launch joyrebot_joint_teleop joint_teleop.launch.py
```

启动时有约 2 秒的 IMU 静置标定(`calibrating(2 seconds)...`),**请把 Joy-Con 水平放在桌上**,标定完再拿起来。手腕通道读 IMU,姿态估计必须先收敛。

在工作区根目录启动,数据日志写到 `./joint_teleop_logs/`。

**上手建议**:先只动手腕(roll/pitch)看 `joint6`/`joint4` 跟随,确认方向对不对;方向反了改 `channel_sign` 对应项为 `-1.0`。熟悉后再加摇杆和按键。

## 手柄输入会话

`joint_teleop_node.py` 只接收标准化的手柄样本并执行关节控制；`joycon_session.py` 独占一个 Joy-Con HID 设备，负责右手柄优先/左手柄回退、摇杆和按键的左右侧绑定、原始报文有效性、输入超时、读取失败后的断连、周期性重连和释放设备。节点不会直接读取 HID 报文或调用 vendor 驱动接口。

## 控制流程

每个控制周期:

```
absolute 通道:  目标[j] = 锚点关节[j] + 符号 × 缩放 × (手柄角 − 锚点手柄角)
rate 通道:      目标[j] = 当前[j] + 符号 × 缩放 × 死区(输入 − 锚点) × dt
指令 = clip(限速(当前 → 目标, max_tracking_speed × dt), 软下限, 软上限)
```

absolute 通道**绑定在锚点上而非逐周期累加**,所以手腕不动关节就绝对不动,误差无法累积。

## 安全机制

- **关节软限位** —— 从 `joyrebot_teleop` 的 `rebot_b601_kinematics.urdf` 读取,各自内缩 `joint_margin`(默认 0.02 rad)。
- **跟随限速** —— 每周期变化不超过 `max_tracking_speed × dt`。这是 IMU 抽风时防止关节猛甩的安全网。
- **输入超时** —— 通过 HID 输入报文是否更新判断手柄是否还在说话;超过 `input_timeout`(0.30 s)保持不动。防的是"摇杆推着不动、手柄突然没电"导致的失控。
- **零报文保护** —— 驱动的报文缓冲区初值是 49 个零字节,零摇杆计数会被解码成满偏 `-1.0`。首个真实报文到达前一律视为无输入,避免启动瞬间关节全速点动。
- **离合无跳变** —— 松开离合时重新锚定,绝对通道不会因为手腕位置变了而跳。
- **启动回 Home** —— 先以 `max_joint_speed` 限速走到 `home_joint_positions` 再接受控制。
- **互斥检查** —— 启动 2 秒后检查 `/rebot/joint1/cmd_pos` 上是否有别的发布者。

## 终端仪表盘

```
┌────────────────────────────────────────────────────┐
│      关节遥操  Right Joy-Con  状态: tracking       │
├────────────────────────────────────────────────────┤
│ 手柄 R  +1.4° P  +1.8° Y  +0.2°   ▲=绝对通道       │
│ 摇杆 H+0.00 V-0.00   R/杆键 +0                     │
├────────────────────────────────────────────────────┤
│ joint1 +0.000 [──────●──────] 余量 2.78            │
│ joint2 +0.300 [─●───────────] 余量 0.28            │
│ joint3 +0.300 [─●───────────] 余量 0.29            │
│▲joint4 -0.002 [──────●──────] 余量 1.55            │
│ joint5 +0.000 [──────●──────] 余量 1.55            │
│▲joint6 +0.003 [──────●──────] 余量 3.12            │
├────────────────────────────────────────────────────┤
│ 夹爪: 开 (0.050 m)   电池: 4/8                     │
└────────────────────────────────────────────────────┘
```

`▲` 标出绝对通道驱动的关节。方括号是关节在软限位区间内的位置,"余量"是距较近一侧限位的弧度。`terminal_display: false` 可关闭。

状态:`tracking` / `hold` / `clutch(冻结)` / `returning_home` / `input_timeout` / `no_joycon`。

## 数据日志

每个控制周期一行,35 列:

| 列 | 含义 |
| --- | --- |
| `ros_time_s`, `status`, `clutch` | 时间戳、状态、离合是否按下 |
| `input_{roll,pitch,yaw,stick_vertical,stick_horizontal,buttons}` | 六个通道的原始输入 |
| `feedback_joint{1..6}` | `/joint_states` 反馈 |
| `velocity_joint{1..6}` | 本周期实际角速度 |
| `command_joint{1..6}` | 发出的关节指令 |
| `command_delta_joint{1..6}` | 相对上一周期的增量 |
| `gripper_normalized`, `gripper_command` | 夹爪归一化状态与实际指令(m) |

日志由独立的 `JointDataLogger` 写入，控制节点只提供每周期的控制数据。默认配置为：

```yaml
data_logging: true
data_log_directory: joint_teleop_logs
data_log_flush_interval: 1.0
```

`data_log_directory` 支持 `~`，相对路径相对于启动命令的当前目录；目录不存在时会自动创建。文件名为 `joint_teleop_YYYYMMDD_HHMMSS.csv`。表头会立刻刷入磁盘，随后按由控制频率换算的约 `data_log_flush_interval` 秒周期刷新；正常退出会刷新并关闭文件。将 `data_logging` 设为 `false` 可完全关闭文件输出。若目录或文件无法创建，节点会输出警告但继续遥操。

## 灵敏度调节

**主旋钮是 `channel_scale`**,在 `config/joint_teleop.yaml` 里。六项一一对应六个通道,顺序固定:

```yaml
#                roll  pitch   yaw  摇杆垂直 摇杆水平  按键
#              →j6    →j4    →j1    →j2     →j5     →j3
channel_scale: [0.75,  0.5,   0.35,  0.25,   0.4,    0.25]
```

`yaw`(第 3 项)刻意是全场最低的。它没有绝对参考、最容易漂,所以用**低增益 + 大死区**(`channel_deadzone` 第 3 项 `0.35`)一起把漂移压住。

**数值越大越灵敏。但两类通道的单位不一样**,这是最容易搞混的地方:

| 通道类型 | 哪几个 | 单位 | 含义 |
| --- | --- | --- | --- |
| `absolute` | roll、pitch | **关节弧度 / 手柄弧度**(无量纲比值) | `1.0` = 手腕转多少关节就转多少;`0.5` = 手腕要转两倍才到位(更迟钝、更好控);`2.0` = 小幅手腕动作对应大幅关节动作 |
| `rate` | yaw、两个摇杆轴、按键 | **满输入时的关节角速度 rad/s** | yaw 的"满输入"= 手腕偏离锚点 1 rad;摇杆 = 推到底;按键 = 按下。`0.25` 就是最快 0.25 rad/s(约 14°/s) |

### 怎么调

一次只改一两项,幅度用**乘除 2**,比小步试错快得多:

| 感觉 | 改哪一项 | 怎么改 |
| --- | --- | --- |
| 整体都太灵敏 | 全部六项 | 一起除以 2 |
| 手腕一动末端就甩 | 第 1、2 项(roll/pitch) | 减半,如 `0.75→0.4`、`0.5→0.25` |
| 手腕要转很大才动到位 | 第 1、2 项 | 加倍 |
| 底座转太快 | 第 3 项(yaw) | 减半 |
| 大臂/小臂动得太急 | 第 4、6 项 | 减半 |
| 腕滚转太快 | 第 5 项 | 减半 |

**辅助旋钮**:

- **手抖/漂移传到关节** → 调大 `channel_deadzone` 对应项(只对 `rate` 通道有效)。`absolute` 通道建议保持 `0.0`,加死区会在中位造出一块"推不动"的死区,跟随手感会变怪。
- **absolute 通道还是太抖** → 这是 IMU 噪声直接传进来了,死区解决不了,只能靠降低 `channel_scale`(噪声会按比例一起缩小)。
- **`max_tracking_speed` 不是灵敏度旋钮** —— 它是 IMU 抽风时的安全上限(默认 3.0 rad/s)。调低会让快速手腕动作发黏、跟不上;正常调灵敏度不要动它。

### 生效方式

改完 `config/joint_teleop.yaml` 后:

```bash
# 不需要重新 build(--symlink-install 已把 config 软链到源码)
# 直接 Ctrl-C 停掉节点再重新启动
ros2 launch joyrebot_joint_teleop joint_teleop.launch.py
```

也可以不改文件、直接在命令行临时试一组值:

```bash
ros2 run joyrebot_joint_teleop joint_teleop --ros-args \
  --params-file install/joyrebot_joint_teleop/share/joyrebot_joint_teleop/config/joint_teleop.yaml \
  -p channel_scale:="[0.4, 0.25, 0.4, 0.15, 0.2, 0.15]"
```

## 其余参数

通道数组顺序同上:**roll, pitch, yaw, 摇杆垂直, 摇杆水平, R/摇杆按下**。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `channel_joint` | `[5,3,0,1,4,2]` | 每个通道绑哪个关节(0=joint1 … 5=joint6) |
| `channel_mode` | `[absolute,absolute,rate,rate,rate,rate]` | 通道语义,可选 `absolute`/`rate`/`off` |
| `channel_sign` | `[1.0,1.0,-1.0,1.0,1.0,1.0]` | 方向反了改符号 |
| `channel_deadzone` | `[0.0,0.0,0.35,0.15,0.15,0.0]` | 见上 |
| `max_tracking_speed` | `3.0` | 安全上限(rad/s),不是灵敏度旋钮 |
| `stick_vertical_sign` / `stick_horizontal_sign` | `1.0` / `-1.0` | 摇杆轴方向,见下方轴名说明 |
| `stick_center` / `stick_half_range` | `2048` / `1400` | 12 位摇杆标定 |
| `joint_margin` | `0.02` | 软限位内缩量(rad) |
| `max_joint_speed` | `0.7` | 仅用于回 Home 的斜坡限速 |

**某个关节不想被控**:把对应通道的 `channel_mode` 改成 `off`。

### ⚠️ 摇杆轴名:vertical = 前后,horizontal = 左右

这两个名字最容易和末端遥操的 X/Y 搞混,进而改错参数:

| 本包参数名 | 摇杆物理方向 | 驱动关节 |
| --- | --- | --- |
| `stick_vertical_sign` | **前后**推 | `joint2` 大臂 |
| `stick_horizontal_sign` | **左右**推 | `joint5` 腕滚转 |

在 `joyrebot_teleop`(末端遥操)里,摇杆前后对应末端 **X** 平移,所以习惯上会把"前后"叫成 X;但本包是关节空间,参数名用的是摇杆自身的 vertical/horizontal。**改方向前先看仪表盘**:

```
│ 摇杆 前后+0.85 左右+0.00   R/杆键 +0                │
```

推一下摇杆,看是"前后"还是"左右"那个数在变,再去改对应的 `_sign`。启动日志里也会打印一次完整绑定:

```
[INFO] 通道绑定:
  手柄roll → joint6  (absolute, scale=0.75, sign=+1)
  手柄pitch → joint4  (absolute, scale=0.5, sign=+1)
  手柄yaw → joint1  (rate, scale=0.35, sign=-1)
  摇杆前后 → joint2  (rate, scale=0.25, sign=+1)
  摇杆左右 → joint5  (rate, scale=0.4, sign=+1)
  R/摇杆按下 → joint3  (rate, scale=0.25, sign=+1)
```

## 测试

```bash
cd src/joyrebot_joint_teleop && python3 -m pytest test/ -q
```

41 项:

- `test_anthropomorphic.py` —— 锚定无跳变、六个通道各自绑对关节(roll→joint6、按键→joint3、摇杆水平→joint5)、绝对通道一比一跟随且不累积、速度通道积分、死区抑制 yaw 抖动、离合重锚不跳变、限速截断、非法通道配置拒绝
- `test_display.py` —— 限位余量计算、CJK 宽度下每行严格等宽(错一格整个框会撕裂)、绝对通道标记
- `test_joint_data_logger.py` —— CSV 表头和行字段顺序、NaN 反馈、刷新周期、关闭和文件创建失败处理
- `test_joycon_session.py` —— 右优先/左回退、摇杆归一化、零报文保护、超时、读取失败断连和释放设备

> 注意:本工作区的 `colcon test` 对 ament_python 包会走 `setup.py test` 而不是 pytest,
> 因此报告 `Ran 0 tests`。这对 `joyrebot_teleop` 同样成立,是既有配置问题。

## 主要文件

| 文件 | 说明 |
| --- | --- |
| `joyrebot_joint_teleop/joint_teleop_node.py` | 唯一 ROS 节点:消费手柄样本、通道映射、发关节指令 |
| `joyrebot_joint_teleop/joycon_session.py` | Joy-Con 获取、摇杆归一化、报文有效性、超时、断连和重连 |
| `joyrebot_joint_teleop/joint_display.py` | 终端仪表盘 |
| `config/joint_teleop.yaml` | 全部参数 |
| `launch/joint_teleop.launch.py` | 只启动这一个节点 |

## 对 joyrebot_teleop 的依赖

不复制驱动和 URDF,直接复用:

- `joyrebot_teleop.vendor.joyconrobotics.JoyconRobotics` —— 手柄连接与 IMU 姿态估计(`get_control()` 返回的 roll/pitch/yaw 与现有末端遥操同源)
- `joyrebot_teleop.kinematics.SerialChain` —— 解析 URDF 取关节限位
- `config/rebot_b601_kinematics.urdf` —— 关节限位来源
