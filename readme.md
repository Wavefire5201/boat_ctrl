# Boat controller for Roboboat Gazebo simulation world
## Installation

1. Clone into src directory
```shell
cd $BOAT_WS/src
git clone https://github.com/Wavefire5201/boat_ctrl
```
2. Build package
```shell
colcon build --merge-install --packages-select boat_ctrl
```
3. Source setup files
```shell
source $BOAT_WS/install/setup.bash
```
4. Run package
```shell
ros2 run boat_ctrl <entry_point>
```
## Entry Points
- controller - control boat
- lidar - view ``PointCloud2`` data
- camera - view front camera output
- buoy - front camera output but with Buoy Detection AI