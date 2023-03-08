# ros_to_labstream
A package for streaming robot state data to LabStream Layer for synchronization with EDA and PPG sensors

Tested on Ubuntu 18.04 running ROS Melodic.

Required Python Packages:
```liblsl```

The single python node subscribes to ```/spot/global_pose``` and ```/hsr/global_pose``` and creates a LabStreamLayer to stream the 6 value robot
state for synchronizatuion by LabRecorder
```
rosrun ros_to_labstream ros_to_labstream.py
```
