# **Unmanned Aerial Vehicle ROS Packages**

Editors: Joshua Caratao

## **Description**
As part of Texas Aerial Robotics (TAR) I have contributed in the development of several ROS packages that allow for the offboard control and autonomous capability of an unmanned aerial vehicle.

Each Folder is a separate ROS Package that should be placed into your catkin_ws/src directory. 

The Controls package deals with interfacing the ROS Nodes with the Pixhawk Flight controller to recieve and publish control commands and implementing control algorithms

The Computer Vision package deals with the detection of Aruco markers and estimation of its pose with respect to the camera frame.

The Simulation_files folder are collection of model files and world files for simulation in gazebo.

## **Hardware Note**
These packages were developed for a UAV platform that uses a pixhawk flight controller running the PX4-autpilot firmware. Using a different flight controller, such as ArduPilot, will prove to be incompatible with the controls package.
Additionally, our drone uses a Nvidia Jetson TX2 paired with an orbitty carrier board as our companion computer and a regular logitech webcam for aruco marker computer vision.


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

  For our purposes, I recommend installing OpenCV for C++, which will take longer, but you can also install OpenCV for Python if you want

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

## 1)Catkin Workspace
Ensure your catkin workspace directory has been made as this is where the ROS packages and scripts are stored.

## 2)PX4-Autopilot Software
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

  You should see a folder called "PX4-Autopilot" in your home directory. We will do more with this later

## 3) Clone this ROS Package Repository

Next, we need to clone and download this repository into your catkin_ws. These packages provide necessary scripts/Nodes for simulating precision positioning over a detected aruco marker 

First, navigate to your catkin workspace directory and into the "src" folder using
```
cd ~/catkin_ws/src
```
This is important as our ROS Packages need to be in this folder within our catkin workspace

After navigating into your catkin workspace, clone this repository into with the following command
```
git clone https://github.com/JoshCaratao/Unmanned-Aerial-Vehicle-ROS-Packages.git 
```
You should now see both of these packages in the "src" folder of your catkin workspace

within your catkin workspace directory, rebuild the workspace and newly installed packages using the following command to ensure all the packages build correctly and that there arent any issues.

```
catkin build
```
  
 







