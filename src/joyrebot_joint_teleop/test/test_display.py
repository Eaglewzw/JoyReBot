import numpy as np
import pytest

from joyrebot_joint_teleop.joint_display import WIDTH, _display_width, limit_headroom, render


def snapshot(**overrides):
    state = dict(
        side="Right Joy-Con", status="tracking",
        names=[f"joint{i}" for i in range(1, 7)],
        command=np.array([0.1, 0.3, 0.35, -0.2, 0.4, 0.0]),
        lower=np.array([-2.78, 0.02, 0.01, -1.55, -1.55, -3.12]),
        upper=np.array([2.78, 3.12, 3.12, 1.55, 1.55, 3.12]),
        absolute={3, 5}, inputs=np.array([0.21, -0.09, 0.03, 0.0, 0.0, 0.0]),
        gripper=1.0, gripper_command=0.05, battery=4)
    state.update(overrides)
    return state


def test_limit_headroom_reports_nearer_limit():
    headroom = limit_headroom([0.0, 0.9, -0.9], [-1.0] * 3, [1.0] * 3)
    assert headroom[0] == pytest.approx(1.0)
    assert headroom[1] == pytest.approx(0.1)
    assert headroom[2] == pytest.approx(0.1)


@pytest.mark.parametrize("overrides", [
    {},
    {"status": "clutch(冻结)", "side": "未连接", "battery": None},
    {"inputs": np.array([-1.5, 1.5, -3.0, -1.0, 1.0, -1.0])},
    {"command": np.array([-2.78, 3.12, 3.12, 1.55, -1.55, -3.12])},   # 紧贴关节限位
])
def test_every_row_is_exactly_the_box_width(overrides):
    """CJK 字符占两个单元格；宽度计算错误会让边框明显错位。"""
    for line in render(snapshot(**overrides)).split("\n"):
        assert _display_width(line) == WIDTH + 2, line


def test_absolute_channels_are_marked():
    rows = render(snapshot(absolute={3, 5})).split("\n")
    marked = [row for row in rows if row.startswith("│▲")]
    assert len(marked) == 2
    assert "joint4" in marked[0] and "joint6" in marked[1]
