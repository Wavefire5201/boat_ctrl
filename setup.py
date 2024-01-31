from setuptools import find_packages, setup

package_name = "jbc"

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
            "controller = jbc.controller:main",
            "camera = jbc.camera:main",
            "buoy = jbc.buoy_recognition:main",
            "lidar = jbc.lidar:main",
            "locate_buoys = jbc.locate_buoys:main",
            "center_of_clusters = jbc.center_of_clusters:main",
            "tasktwo = jbc.tasktwo:main",
            "average_buoy_location = jbc.average_buoy_location:main",
            "detect_radius = jbc.detect_radius:main",
            "boat_taskone_waypoint = jbc.taskone:main",
        ],
    },
)
