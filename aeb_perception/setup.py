from setuptools import find_packages, setup

package_name = "aeb_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Beni",
    maintainer_email="darabosbeni@gmail.com",
    description=(
        "LiDAR -> CenterPoint (MMDetection3D) -> autoware_perception_msgs perception "
        "node, plusz adapter node a braking_system fele."
    ),
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = aeb_perception.perception_node:main",
            "adapter_node = aeb_perception.adapter_node:main",
	    "detection_visualizer = aeb_perception.detection_visualizer:main",
        ],
    },
)
