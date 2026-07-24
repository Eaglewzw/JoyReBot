"""与 ROS 无关的拟人化关节遥操 CSV 会话日志。"""

import csv
from datetime import datetime
from pathlib import Path


class JointDataLogger:
    """以固定 CSV 字段结构写入关节遥操遥测数据。"""

    def __init__(self, enabled, directory, control_rate, flush_interval,
                 channel_names, joint_names):
        self.enabled = bool(enabled)
        self.directory = Path(directory).expanduser()
        self.channel_names = list(channel_names)
        self.joint_names = list(joint_names)
        self.flush_every = max(1, int(float(control_rate) * float(flush_interval)))
        self.file = None
        self.writer = None
        self.rows_since_flush = 0

    def open(self):
        """日志启用时打开时间戳 CSV 文件，并返回其路径。"""
        if not self.enabled:
            return None
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / f"joint_teleop_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        columns = ["ros_time_s", "status", "clutch"]
        columns += [f"input_{name}" for name in self.channel_names]
        for prefix in ("feedback", "velocity", "command", "command_delta"):
            columns += [f"{prefix}_{name}" for name in self.joint_names]
        columns += ["gripper_normalized", "gripper_command"]
        self.writer.writerow(columns)
        self.file.flush()
        return path

    def write(self, *, ros_time_s, status, clutch, inputs, feedback, velocity,
              command, command_delta, gripper_normalized, gripper_command):
        """日志启用且已打开文件时，追加一条完整控制周期记录。"""
        if self.writer is None:
            return
        if feedback is None:
            feedback = [float("nan")] * len(self.joint_names)
        row = [ros_time_s, status, int(clutch)]
        row += list(inputs)
        row += list(feedback)
        row += list(velocity)
        row += list(command)
        row += list(command_delta)
        row += [gripper_normalized, gripper_command]
        self.writer.writerow(row)
        self.rows_since_flush += 1
        if self.rows_since_flush >= self.flush_every:
            self.file.flush()
            self.rows_since_flush = 0

    def close(self):
        """刷新并关闭当前输出流；可重复调用。"""
        if self.file is None:
            return
        self.file.flush()
        self.file.close()
        self.file = None
        self.writer = None
        self.rows_since_flush = 0
