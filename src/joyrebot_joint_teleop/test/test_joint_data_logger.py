import csv
import math

import pytest

from joyrebot_joint_teleop.joint_data_logger import JointDataLogger


CHANNEL_NAMES = ("roll", "pitch", "yaw", "stick_vertical", "stick_horizontal", "buttons")
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
EXPECTED_HEADER = [
    "ros_time_s", "status", "clutch",
    "input_roll", "input_pitch", "input_yaw", "input_stick_vertical",
    "input_stick_horizontal", "input_buttons",
    "feedback_joint1", "feedback_joint2", "feedback_joint3", "feedback_joint4",
    "feedback_joint5", "feedback_joint6",
    "velocity_joint1", "velocity_joint2", "velocity_joint3", "velocity_joint4",
    "velocity_joint5", "velocity_joint6",
    "command_joint1", "command_joint2", "command_joint3", "command_joint4",
    "command_joint5", "command_joint6",
    "command_delta_joint1", "command_delta_joint2", "command_delta_joint3",
    "command_delta_joint4", "command_delta_joint5", "command_delta_joint6",
    "gripper_normalized", "gripper_command",
]


def make_logger(directory, enabled=True, control_rate=2.0, flush_interval=1.0):
    return JointDataLogger(enabled, directory, control_rate, flush_interval,
                           CHANNEL_NAMES, JOINT_NAMES)


def write_sample(logger, feedback=(10, 11, 12, 13, 14, 15)):
    logger.write(
        ros_time_s=123.5,
        status="tracking",
        clutch=True,
        inputs=(1, 2, 3, 4, 5, 6),
        feedback=feedback,
        velocity=(20, 21, 22, 23, 24, 25),
        command=(30, 31, 32, 33, 34, 35),
        command_delta=(40, 41, 42, 43, 44, 45),
        gripper_normalized=1.0,
        gripper_command=0.05,
    )


def test_disabled_logger_creates_no_output(tmp_path):
    directory = tmp_path / "logs"
    logger = make_logger(directory, enabled=False)

    assert logger.open() is None
    logger.write(ros_time_s=0.0, status="hold", clutch=False, inputs=(), feedback=None,
                 velocity=(), command=(), command_delta=(), gripper_normalized=0.0,
                 gripper_command=0.0)
    logger.close()

    assert not directory.exists()


def test_open_writes_exact_header_and_row_order(tmp_path):
    logger = make_logger(tmp_path / "nested" / "logs")
    path = logger.open()

    assert path.parent == tmp_path / "nested" / "logs"
    assert path.name.startswith("joint_teleop_")
    assert path.suffix == ".csv"
    with path.open(newline="", encoding="utf-8") as stream:
        assert next(csv.reader(stream)) == EXPECTED_HEADER

    write_sample(logger)
    logger.close()
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))

    assert rows[1] == [
        "123.5", "tracking", "1", "1", "2", "3", "4", "5", "6",
        "10", "11", "12", "13", "14", "15",
        "20", "21", "22", "23", "24", "25",
        "30", "31", "32", "33", "34", "35",
        "40", "41", "42", "43", "44", "45", "1.0", "0.05",
    ]


def test_missing_feedback_uses_nan_and_close_is_idempotent(tmp_path):
    logger = make_logger(tmp_path / "logs")
    path = logger.open()

    write_sample(logger, feedback=None)
    logger.close()
    logger.close()

    with path.open(newline="", encoding="utf-8") as stream:
        row = list(csv.reader(stream))[1]
    assert all(math.isnan(float(value)) for value in row[9:15])


def test_flushes_after_configured_record_count(tmp_path):
    logger = make_logger(tmp_path / "logs", control_rate=2.0, flush_interval=1.0)
    path = logger.open()

    write_sample(logger)
    assert logger.rows_since_flush == 1
    write_sample(logger)
    assert logger.rows_since_flush == 0

    with path.open(newline="", encoding="utf-8") as stream:
        assert len(list(csv.reader(stream))) == 3
    logger.close()


def test_open_propagates_os_error(tmp_path, monkeypatch):
    logger = make_logger(tmp_path / "logs")

    def fail(*args, **kwargs):
        raise OSError("unwritable")

    monkeypatch.setattr(type(logger.directory), "mkdir", fail)
    with pytest.raises(OSError, match="unwritable"):
        logger.open()
