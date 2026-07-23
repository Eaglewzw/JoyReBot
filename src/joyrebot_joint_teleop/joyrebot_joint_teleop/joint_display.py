"""Terminal dashboard for anthropomorphic joint control."""

import math
import unicodedata

import numpy as np


WIDTH = 52
BAR_WIDTH = 13


def limit_headroom(position, lower, upper):
    """Distance to the nearer soft limit for every joint."""
    position = np.asarray(position, dtype=float)
    return np.minimum(position - np.asarray(lower), np.asarray(upper) - position)


def _display_width(text):
    # Same convention as joyrebot_teleop.terminal_display: W/F CJK glyphs take
    # two cells, ambiguous characters such as the degree sign take one.
    return sum(2 if unicodedata.east_asian_width(char) in ("W", "F") else 1 for char in text)


def _pad(text, width=WIDTH):
    return text + " " * max(0, width - _display_width(text))


def _center(text, width=WIDTH):
    remaining = max(0, width - _display_width(text))
    return " " * (remaining // 2) + text + " " * (remaining - remaining // 2)


def _bar(value, lower, upper, width=BAR_WIDTH):
    """Where the joint sits inside its soft limits."""
    span = max(1e-6, upper - lower)
    slot = int(round((value - lower) / span * (width - 1)))
    slot = max(0, min(width - 1, slot))
    return "[" + "─" * slot + "●" + "─" * (width - 1 - slot) + "]"


def _rule():
    return "├" + "─" * WIDTH + "┤"


def render(state):
    """Build the dashboard from the snapshot dict assembled by the node."""
    headroom = limit_headroom(state["command"], state["lower"], state["upper"])
    roll, pitch, yaw = (math.degrees(value) for value in state["inputs"][:3])
    vertical, horizontal, buttons = state["inputs"][3:6]
    rows = ["┌" + "─" * WIDTH + "┐",
            "│" + _center(f"关节遥操  {state['side']}  状态: {state['status']}") + "│",
            _rule(),
            "│" + _pad(f" 手柄 R{roll:+6.1f}° P{pitch:+6.1f}° Y{yaw:+6.1f}°"
                       f"   ▲=绝对通道") + "│",
            "│" + _pad(f" 摇杆 前后{vertical:+5.2f} 左右{horizontal:+5.2f}"
                       f"   R/杆键 {buttons:+.0f}") + "│",
            _rule()]
    for index, name in enumerate(state["names"]):
        marker = "▲" if index in state["absolute"] else " "
        rows.append("│" + _pad(
            f"{marker}{name} {state['command'][index]:+6.3f} "
            f"{_bar(state['command'][index], state['lower'][index], state['upper'][index])} "
            f"余量{headroom[index]:5.2f}") + "│")
    rows.append(_rule())
    battery = "--" if state["battery"] is None else f"{state['battery']}/8"
    rows.append("│" + _pad(f" 夹爪: {'开' if state['gripper'] > 0.5 else '闭'} "
                           f"({state['gripper_command']:.3f} m)   电池: {battery}") + "│")
    rows.append("└" + "─" * WIDTH + "┘")
    return "\n".join(rows)


def print_dashboard(state):
    # No ANSI clear/home sequences: ros2 launch prefixes each completed line and
    # an escape before the first line would shift the box.
    print(render(state), flush=True)
