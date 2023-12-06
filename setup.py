from setuptools import find_packages, setup

package_name = "boat_ctrl"

setup(
    name=package_name,
    version="0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wavefire5201",
    maintainer_email="enoch.zhu154@gmail.com",
    description="Multitool package for roboboat simulation",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = boat_ctrl.controller:main",
            "camera = boat_ctrl.camera:main",
            "buoy = boat_ctrl.buoy_recognition:main",
            "lidar = boat_ctrl.lidar:main",
            "taskone = boat_ctrl.taskone:main",
            "tasktwo = boat_ctrl.tasktwo:main",
            "locate_buoys = boat_ctrl.locate_buoys:main",
            "center_of_clusters = boat_ctrl.center_of_clusters:main",
            "buoy_jonathan = boat_ctrl.buoy_recognition_jonathan:main", 
        ],
    },
)
