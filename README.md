# ros_to_labstream
A package for streaming robot state data to LabStream Layer for synchronization with EDA and PPG sensors

Tested on Ubuntu 18.04 running ROS Melodic and python 2.7.

To get it working on the hcrl nuc:
had to ensure the correct ```pip --version``` was being used:
```
pip 20.3.4 from /home/hcrl-nuc/.local/lib/python2.7/site-packages/pip (python 2.7)
```
was my output.
Then we run
```
pip install pylsl==1.14
```

Required Python Packages:
```pylsl```

The single python node subscribes to ```/gcr_spot/localization_ros``` and ```/go1/localization_ros``` and creates a LabStreamLayer to stream the 6 value robot
state for synchronizatuion by LabRecorder
```
rosrun ros_to_labstream ros_to_labstream.py
```
OR
```
rosrun ros_to_labstream try2.py
```
