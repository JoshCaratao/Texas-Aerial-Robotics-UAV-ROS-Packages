# **Texas Aerial Robotics (TAR) UAV ROS Packages**

Editors: Joshua Caratao

## **Description**
As part of Texas Aerial Robotics (TAR) Drone Group 2 my team and I have contributed in the development of several ROS packages that allow for the offboard control and autonomous capability of an unmanned aerial vehicle through integration with the PX4 Flight stack.

Each Folder is a separate ROS Package that should be placed into your catkin_ws/src directory. (Directions for installing these packages are in the simulation documentation below) 

### Controls Package
The Controls package deals with interfacing the ROS Nodes with the Pixhawk Flight controller to recieve and publish control commands and implementing control algorithms. Currently, we are working on programming scripts for precision positioning over an aruco marker through the implementation of a Proportional-Integral-Derivative (PID) Controller. 
To validate this control system for precision positioning and landing, we have also developed scripts for integrating this control into a Gazebo Software-In-The-Loop (SITL) simulation environment that involves a drone, camera, computer vision scripts, aruco markers, our control system, and the PX4 flight stack, allowing us to validate and test our precision control system prior to more dangerous physical testing.

### Computer Vision Package
The Computer Vision package deals with interfacing the ROS system with the drone camera hardware and detection of Aruco markers and estimation of its pose with respect to the camera frame. Just like with the Controls Package, we have also developed scripts specifically for SITL simulation in the Gazebo enviroment, allowing us to simulate the aruco marker detection virtually.

## **Hardware Note**
These packages were developed for a UAV platform that uses a pixhawk flight controller running the PX4-autpilot firmware. Using a different flight controller, such as ArduPilot, will prove to be incompatible with the controls package.
Additionally, our drone uses a Nvidia Jetson TX1 paired with an orbitty carrier board as our companion computer and a regular logitech webcam for aruco marker computer vision.


## **Software Note (Package Dependencies)**
Because of the Jetson TX2 support, we were limited to Ubuntu 18.04 OS and therefore, to developing these packages in ROS Melodic.
Other Package dependencies include MAVROS, Mavlink, and OpenCV(c++ and Python versions) which have to be installed and built separately first.

## **Ubuntu and ROS Software Installations**
### 1. Have a proper installation of Ubuntu 18.04 (Bionic)
  Before you can install and use ROS, you need a compatible Operating System. For ROS melodic (the ROS version I am using), you can use Ubuntu 18.04. Ubuntu is a popular open-source Linux distro based on Debian and can be installed from resources online.
  
  Using Ubuntu could be done either through a dedicated linux machine, dual booting a laptop/PC, or using a USB Bootdrive with Ubuntu 18.04

### 2. Install and build ROS Melodic

  Robot Operating System (ROS), despite its name, ROS is not actually an operating system, but is essentially a framework that developers can use to build and manage complex robotic systems. It uses a subscriber/publisher messaging system that involves different nodes (scripts/programs) communicating to each other through "ros topics." These nodes can publish data, subscribe, or do both to different message topics. This allows you to modularize your system, swapping out or swapping in differnt nodes without having to worry too much about affecting your other programs. It also allows for your programs to run in parallel.
  
  Once you are sure your system meets the minimum requirements for ROS Melodic (Ubuntu 18.04), you can continue with the installation of ROS Melodic
  Follow the instructions in the following link: https://wiki.ros.org/melodic/Installation/Ubuntu

*Note* if you are unable to install the Desktop-Full version, you can install a different version and install other packages/dependencies as you need them

### 3. Installation of OpenCV

  OpenCV is a popular open source Computer Vision library useful for image collection, image and video processing, and computer vision tasks (object recognition, aruco marker detection, etc). OpenCV is written in C++ but has bindings for other languages like Python.
  
  This will be a critical component of our drone as OpenCV will allow us to interface cameras with our ROS program and perform tasks like Aruco Marker Detection/Pose Estimation and Simultaneous Localization and Mapping (SLAM).

  For our purposes, I recommend installing OpenCV for C++, which will take longer, but you can also install OpenCV for Python if you want.

  NOTE: OpenCV may already be installed as part of the ROS installations (I believe depending on which ROS installation size option you choose). Thus, installing OpenCV again may introduce some conflict errors down the road. However, this can be fixed by specifying the OpenCV version you want to use within the "CMakeLists.txt" file of your ROS packages.

#### **OpenCV with C++**

  The installation can be completed by following the link below. Highly highly recommend following the "install from source" option. 
    https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

*IMPORTANT NOTES BEFORE INSTALLING*

If you do not need the contrib_module files then on step 2, do not clone the “opencv_contrib repository, also ensure to exclude “-D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \” when running the cmake command:  

If you are installing this on the Jetson TX2 already, Make –j4 instead of –j8 this is due to the TX2 only having 4 cores. 

If you wish to install an older version of OPENCV use git checkout as mentioned. Version 4.7 has worked so far on with Ubuntu 18.04, ROS Melodic**

*Checking if OpenCV is installed*


Create a C++ file and copy the following


    #include <opencv2/opencv.hpp>
    int main() {
      std::cout<<"The current OpenCV version is" << CV_VERSION << "\n";
      return 0;
    }

After saving the file, in the terminal go to the files directory and paste the following command to compile the C++ code. *(Replace "your_program_file" with the name of your file)*

    g++ your_program_file.cpp -o your_program_file `pkg-config --cflags --libs opencv4`

After compiling, run the code using the following command

    ./your_program_file

The output should be similar to the following

    The current OpenCV version is 4.8.0

  
# Running Offboard Control Simulations in Gazebo
If you have already completed the steps above and want to run offboard control simulations involving aruco marker detection and positioning of the drone precisely over an aruco marker through a simulated downward facing drone camera, you can follow these steps below.

NOTE: Offboard control refers to control of the UAV/Drone fully autonomously through ROS and the companion computer rather than through a Ground Control Application or remote controller. 

## 1) Catkin Workspace
Ensure your catkin workspace directory has been made as this is where the ROS packages and scripts are stored.

## 2) PX4-Autopilot Software
Download and install the PX4-Autopilo software. This is crucial for running software-in-the-loop (SITL) simulations of our drone and is necesarry for our control package simulations to work correctly.

Use the link below and follow the instructions for "Download the PX4 Code"
```
   https://docs.px4.io/main/en/dev_setup/building_px4.html
```
  Alternatively, just copy and paste this command into your terminal while in your home directory

```
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
  This will clone the very latest ("main") version of the PX4-Autopilot Repository into the current directory you are in.

  You should see a folder called "PX4-Autopilot" in your home directory.
  ### NOTE: you may run into issues with OpenCV later. 
  If you run into OpenCV Version conflict issues, you will need to add a line into the CMakeLists.txt file of the "sitl_gazebo-classic" folder in the PX4-Autopilot folder to specify the version of OpenCV you want to use (assuming the conflict is due to having more than 1 version installed). If you need to do this, navigate to the proper directory
```
cd /home/ubuntu/PX4-Autopilot/Tools/simulation/gazebo-classic
``` 
within this directory, open the CMakeLists.txt file in your code editor and add the following line under "find_package(gazebo REQUIRED)"
NOTE: For my use, I specified to use version 3.2.0 but you may choose to specify the version on your own system.

```
find_package (OpenCV 3.2.0 REQUIRED)
```
This should ensure the the simulation uses the OpenCV version installed from your ROS installation rather than an OpenCV version you may have installed system-wide.

## 3) Clone and install MAVROS and MAVlink Packages
These packages are necessary for our computer/jetson to communicate and interact with our pixhawk/flghtcontroller. MAVROS acts as a bridge between our ROS system, translating our ROS messages into MAVLink messages that our pixhawk and PX4-Autopilot software can understand.

Follow this following tutorial for installing ROS with MAVROS through "Source Installation"
```
https://docs.px4.io/main/en/ros/mavros_installation.html
```
Make sure to rebuild your catkin workspace after installing these by running the following command in your catkin workspace

```
catkin build
```

## 4) Clone this Texas Aerial Robotics ROS Package Repository

Next, we need to clone and download this Texas Aerial Robotics repository into your catkin_ws. As mentioned above these packages provide the necessary scripts/Nodes for simulating precision positioning control over a detected aruco marker

First, navigate to your catkin workspace directory and into the "src" folder using
```
cd ~/catkin_ws/src
```
This is important, as our ROS Packages need to be in this folder within our catkin workspace

After navigating into your catkin workspace, clone this repository with the following command
```
git clone https://github.com/JoshCaratao/Texas-Aerial-Robotics-UAV-ROS-Packages.git 
```
You should now see both of these packages in the "src" folder of your catkin workspace

within your catkin workspace directory, rebuild the workspace and newly installed packages using the following command to ensure all the packages build correctly and that there aren't any issues.

```
catkin build
```

## 5) Download Simulation models/files

In order to simulate a drone with a downward facing camera and a world with aruco markers, we need to download the following files and place them in the proper directories.

Clone my "UAV Simulation Files" Repository into your home directory (doesn't really need to be in the catkin workspace as we will be moving these after downloading them).

```
git clone https://github.com/JoshCaratao/Texas-Aerial-Robotics-UAV-Simulation-Files.git
```

Included are a variety of gazebo model files, world file and a launch file necessary for our simulations. These will all need to be copied and pasted into proper directories for the simulation to run properly.

CONTINUE HERE, NOT FINISHED YET




  
 







