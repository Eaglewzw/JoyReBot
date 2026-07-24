import pytest

from joyrebot_joint_teleop.joycon_session import (
    JoyconSession, normalize_axis, report_is_ready,
)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeJoycon:
    def __init__(self, report=b"\x30", horizontal=2048, vertical=2048, buttons=None):
        self._input_report = report
        self.horizontal = horizontal
        self.vertical = vertical
        self.buttons = buttons or {}
        self.battery = 4

    def get_stick_right_horizontal(self):
        return self.horizontal

    def get_stick_right_vertical(self):
        return self.vertical

    def get_stick_left_horizontal(self):
        return self.horizontal

    def get_stick_left_vertical(self):
        return self.vertical

    def get_battery_level(self):
        return self.battery

    def __getattr__(self, name):
        if name.startswith("get_button_"):
            return lambda: self.buttons.get(name, False)
        raise AttributeError(name)


class FakeController:
    def __init__(self, side, joycon=None, fail_read=False, **kwargs):
        self.side = side
        self.joycon = joycon or FakeJoycon()
        self.fail_read = fail_read
        self.running = True
        self.disconnected = False
        self.options = kwargs

    def get_control(self):
        if self.fail_read:
            raise RuntimeError("read failure")
        return ([0.0, 0.0, 0.0, 0.1, 0.2, 0.3], None, None)

    def disconnnect(self):
        self.disconnected = True


def factory_for(available, created):
    def factory(side, **kwargs):
        created.append((side, kwargs))
        value = available[side]
        if isinstance(value, Exception):
            raise value
        return value
    return factory


def make_session(available, clock=None, created=None, **kwargs):
    clock = clock or Clock()
    created = created if created is not None else []
    return JoyconSession(
        input_timeout=0.30, stick_center=2048.0, stick_half_range=1400.0,
        stick_horizontal_sign=-1.0, stick_vertical_sign=1.0, clock=clock,
        controller_factory=factory_for(available, created), **kwargs), clock, created


def test_normalize_axis_centres_and_saturates():
    assert normalize_axis(2048, 2048.0, 1400.0) == pytest.approx(0.0)
    assert normalize_axis(2048 + 700, 2048.0, 1400.0) == pytest.approx(0.5)
    assert normalize_axis(4095, 2048.0, 1400.0) == pytest.approx(1.0)
    assert normalize_axis(0, 2048.0, 1400.0) == pytest.approx(-1.0)


def test_zero_filled_report_is_rejected():
    # 全零缓冲区会被解码为负向满偏，因此绝不能作为有效输入。
    assert not report_is_ready(bytes(49))
    assert not report_is_ready(b"")
    assert normalize_axis(0, 2048.0, 1400.0) == pytest.approx(-1.0)
    assert report_is_ready(b"\x30" + bytes(48))


def test_connect_prefers_right_and_keeps_vendor_options():
    right, left = FakeController("right"), FakeController("left")
    session, _, created = make_session({"right": right, "left": left})

    assert session.connect()
    assert session.side == "right"
    assert created == [("right", {
        "without_rest_init": False, "all_button_return": True,
        "gripper_open": 1.0, "gripper_close": 0.0,
        "enable_shoulder_translation": True,
    })]


def test_connect_falls_back_to_left_and_stays_disconnected_when_unavailable():
    left = FakeController("left")
    session, _, created = make_session({"right": RuntimeError("missing"), "left": left})

    assert session.connect()
    assert session.side == "left"
    assert [side for side, _ in created] == ["right", "left"]

    missing, _, _ = make_session({"right": RuntimeError("missing"), "left": RuntimeError("missing")})
    assert not missing.connect()
    assert not missing.connected


def test_poll_normalizes_right_sample_and_semantic_buttons():
    joycon = FakeJoycon(
        horizontal=3448, vertical=648,
        buttons={"get_button_zr": True, "get_button_r": True, "get_button_b": True},
    )
    session, _, _ = make_session({"right": FakeController("right", joycon), "left": None})
    session.connect()

    sample = session.poll()

    assert sample.fresh
    assert sample.side == "right"
    assert (sample.roll, sample.pitch, sample.yaw) == (0.1, 0.2, 0.3)
    assert sample.stick_horizontal == -1.0
    assert sample.stick_vertical == -1.0
    assert sample.buttons == {
        "gripper": True, "shoulder": True, "stick_press": False,
        "clutch": True, "reanchor": False, "home": False,
    }


def test_poll_uses_left_bindings():
    joycon = FakeJoycon(buttons={"get_button_zl": True, "get_button_l": True, "get_button_down": True})
    session, _, _ = make_session({"right": RuntimeError("missing"), "left": FakeController("left", joycon)})
    session.connect()

    sample = session.poll()

    assert sample.fresh
    assert sample.side == "left"
    assert sample.buttons["gripper"]
    assert sample.buttons["shoulder"]
    assert sample.buttons["clutch"]


def test_zero_report_and_timeout_are_nonfresh_without_disconnect():
    clock = Clock()
    joycon = FakeJoycon(report=bytes(49))
    controller = FakeController("right", joycon)
    session, _, _ = make_session({"right": controller, "left": None}, clock)
    session.connect()

    assert not session.poll().fresh
    assert session.connected

    joycon._input_report = b"\x30"
    assert session.poll().fresh
    clock.now = 0.30
    assert session.poll().fresh
    clock.now = 0.31
    stale = session.poll()
    assert not stale.fresh
    assert stale.connected
    assert stale.buttons is not None

    joycon._input_report = b"\x31"
    assert session.poll().fresh


def test_read_failure_disconnects_and_close_is_idempotent():
    warnings = []
    controller = FakeController("right", fail_read=True)
    session, _, _ = make_session(
        {"right": controller, "left": None}, warning=warnings.append)
    session.connect()
    previous_generation = session.connection_generation

    sample = session.poll()

    assert not sample.connected
    assert not sample.fresh
    assert controller.disconnected
    assert not controller.running
    assert session.connection_generation == previous_generation + 1
    assert warnings == ["Joy-Con read failed (read failure); dropping the connection"]
    session.close()
    session.close()
