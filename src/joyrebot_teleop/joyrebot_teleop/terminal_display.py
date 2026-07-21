"""Terminal dashboard formatting for one or two Joy-Con data sources."""

import unicodedata


WIDTH = 34


def _display_width(text):
    # Modern UTF-8 terminals render W/F CJK glyphs as two cells, while
    # ambiguous characters such as the degree sign are normally one cell.
    return sum(2 if unicodedata.east_asian_width(char) in ("W", "F") else 1 for char in text)


def _center(text, width=WIDTH):
    remaining = max(0, width - _display_width(text))
    return " " * (remaining // 2) + text + " " * (remaining - remaining // 2)


def _pad(text, width=WIDTH):
    return text + " " * max(0, width - _display_width(text))


def snapshot(controller, posture):
    status = controller.joycon.get_status()
    pressed = []
    for group in ("left", "shared", "right"):
        pressed.extend(name.upper() for name, value in status["buttons"][group].items() if value)
    sticks = status["analog-sticks"]
    battery = status["battery"]
    return {
        "buttons": ",".join(pressed) if pressed else "(无)",
        "left_stick": (sticks["left"]["horizontal"], sticks["left"]["vertical"]),
        "right_stick": (sticks["right"]["horizontal"], sticks["right"]["vertical"]),
        "rpy": tuple(value * 180.0 / 3.141592653589793 for value in posture[3:]),
        "position": tuple(posture[:3]),
        "battery": int(battery["level"]),
        "charging": bool(battery["charging"]),
    }


def _lines(data):
    if data is None:
        return ["状态: 未连接", "摇杆: --", "Roll : --", "Pitch: --", "Yaw  : --", "位置: --", "电池: --"]
    left, right = data["left_stick"], data["right_stick"]
    roll, pitch, yaw = data["rpy"]
    x, y, z = data["position"]
    charging = " 充电" if data["charging"] else ""
    return [
        f"按键: {data['buttons']}",
        f"摇杆: L({left[0]},{left[1]}) R({right[0]},{right[1]})",
        f"Roll : {roll:+8.1f}°",
        f"Pitch: {pitch:+8.1f}°",
        f"Yaw  : {yaw:+8.1f}°",
        f"位置: X{x:+.3f} Y{y:+.3f} Z{z:+.3f}",
        f"电池: {data['battery']}/8{charging}",
    ]


def render(left=None, right=None):
    left_lines, right_lines = _lines(left), _lines(right)
    rows = ["┌" + "─" * WIDTH + "┬" + "─" * WIDTH + "┐",
            "│" + _center("左手柄 (Left)") + "│" + _center("右手柄 (Right)") + "│",
            "├" + "─" * WIDTH + "┼" + "─" * WIDTH + "┤"]
    rows.extend("│" + _pad(a) + "│" + _pad(b) + "│" for a, b in zip(left_lines, right_lines))
    rows.append("└" + "─" * WIDTH + "┴" + "─" * WIDTH + "┘")
    return "\n".join(rows)


def print_dashboard(left=None, right=None):
    # Do not emit ANSI clear/home sequences. ros2 launch captures multiline
    # stdout and prefixes each completed line; an escape sequence before the
    # first line makes only that line lose the process prefix and shifts the box.
    print(render(left, right), flush=True)
