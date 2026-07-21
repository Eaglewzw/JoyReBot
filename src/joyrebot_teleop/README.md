# joyrebot_teleop

ROS 2 Cartesian teleoperation for the reBot B601-RS simulation. The runtime
core of `joycon-robotics` is included under `joyrebot_teleop/vendor`, so a
second source repository is not required. Commands are generated only while the
clutch is held, are solved from the current joint state, and are constrained by
workspace, joint-limit, rate and input-timeout checks.

## Dependencies

Python dependencies `hidapi` and `PyGLM` are declared by this package. The
Nintendo kernel driver, Bluetooth pairing and udev permissions must still be
configured on the host because they cannot be embedded in a Python package.
Mock mode does not require a Joy-Con.

## Build and run

From the JoyReBot workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select joyrebot_gazebo_sim joyrebot_teleop
source install/setup.bash
ros2 launch joyrebot_teleop sim_teleop.launch.py
```

For a hardware-free smoke test:

```bash
ros2 launch joyrebot_teleop sim_teleop.launch.py mock:=true
```

Hold `SL` to engage motion, release it to freeze/re-anchor, use `ZR` to toggle
the gripper, and use `+` to recalibrate the controller. Place the Joy-Con still
on a horizontal surface during its initial calibration.

Tune coordinate signs, scaling, workspace and speed in `config/teleop.yaml`.
Always validate a new mapping in Gazebo before connecting a physical arm.
