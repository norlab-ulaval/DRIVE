from setuptools import setup

package_name = "drive_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="norlab",
    maintainer_email="you@example.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_ros_bridge = drive_ros.drive_ros_bridge:main",
            "diff_drive_sim = drive_ros.diff_drive_sim:main",
            "keyboard_teleop = drive_ros.keyboard_teleop:main",
        ],
    },
)
