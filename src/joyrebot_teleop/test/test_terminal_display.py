from joyrebot_teleop.terminal_display import render


def test_dashboard_contains_controller_data():
    data = {
        "buttons": "SL,ZR", "left_stick": (2048, 2047),
        "right_stick": (2051, 2044), "rpy": (3.2, 4.0, -6.5),
        "position": (0.304, 0.272, -2.357), "battery": 2, "charging": False,
    }
    output = render(right=data)
    assert "左手柄 (Left)" in output
    assert "右手柄 (Right)" in output
    assert "按键: SL,ZR" in output
    assert "R(2051,2044)" in output
    assert "Yaw  :     -6.5°" in output
    assert "电池: 2/8" in output
