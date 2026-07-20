from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def test_urdf_has_expected_joints_and_meshes():
    root = ET.parse(PACKAGE / "urdf" / "rebot_b601_rs.urdf").getroot()
    joints = {joint.attrib["name"] for joint in root.findall("joint")}
    assert {f"joint{i}" for i in range(1, 7)} <= joints
    assert {"joint_left", "joint_right"} <= joints
    for mesh in root.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        assert uri.startswith("package://joyrebot_gazebo_sim/meshes/")
        assert (PACKAGE / "meshes" / Path(uri).name).is_file()


def test_world_matches_mujoco_scene_objects():
    root = ET.parse(PACKAGE / "worlds" / "rebot_b601.sdf").getroot()
    assert root.attrib["version"] == "1.10"
    models = {model.attrib["name"] for model in root.findall(".//world/model")}
    assert {"robot_lab_room", "tool_cabinet", "table", "cube"} <= models
    assert root.find(".//world/scene/sky") is not None


def test_launch_explicitly_targets_harmonic():
    launch = (PACKAGE / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    assert '"gz_version": "8"' in launch
    assert "IGN_GAZEBO_RESOURCE_PATH" not in launch


def test_joint3_controller_can_lift_distal_chain():
    root = ET.parse(PACKAGE / "urdf" / "rebot_b601_rs.urdf").getroot()
    controllers = root.findall(".//gazebo/plugin")
    joint3 = next(
        plugin for plugin in controllers
        if plugin.findtext("joint_name") == "joint3"
    )
    assert float(joint3.findtext("p_gain")) >= 80.0
    assert float(joint3.findtext("d_gain")) >= 5.0
    assert float(joint3.findtext("cmd_max")) == 36.0
