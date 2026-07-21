from setuptools import find_packages, setup

package_name = "joyrebot_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/teleop.yaml"]),
        ("share/" + package_name + "/launch", ["launch/teleop.launch.py", "launch/sim_teleop.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "scipy", "hidapi", "PyGLM"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "joycon_input = joyrebot_teleop.joycon_input_node:main",
        "mock_input = joyrebot_teleop.mock_input_node:main",
        "teleop_controller = joyrebot_teleop.teleop_controller:main",
    ]},
)
