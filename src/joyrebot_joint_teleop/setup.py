from setuptools import find_packages, setup

package_name = "joyrebot_joint_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/joint_teleop.yaml"]),
        ("share/" + package_name + "/launch", ["launch/joint_teleop.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "hidapi"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "joint_teleop = joyrebot_joint_teleop.joint_teleop_node:main",
    ]},
)
