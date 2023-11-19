# **Unmanned Aerial Vehicle ROS Packages**

## **Description**
As part of Texas Aerial Robotics (TAR) I have contributed in the development of several ROS packages that allow for the offboard control and autonomous capability of an unmanned aerial vehicle.

Each Folder is a separate ROS Package. 

The Controls package deals with interfacing the ROS Nodes with the Pixhawk Flight controller to recieve and publish control commands and implementing control algorithms

The Computer Vision package deals with the detection of Aruco markers and estimation of its pose with respect to the camera frame.

## **Hardware Note**
These packages were developed for a UAV platform that uses a pixhawk flight controller running the PX4-autpilot firmware. Using a different flight controller, such as ArduPilot, will prove to be incompatible with the controls package.
Additionally, our drone uses a Nvidia Jetson TX2 paired with an orbitty carrier board as our companion computer and a regular logitech webcam for aruco marker computer vision.


## **Software Note (Package Dependencies)**
Because of the Jetson TX2 support, we were limited to Ubuntu 18.04 OS and therefore, to developing these packages in ROS Melodic.
Other Package dependencies include MAVROS, Mavlink, and OpenCV(c++ and Python versions) which have to be installed and built separately first.
