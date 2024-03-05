# **Texas Aerial Robotics (TAR) UAV ROS Packages**

Editors: Joshua Caratao

Last Updated: 3/4/2024

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
![DroneSim3](https://github.com/JoshCaratao/Texas-Aerial-Robotics-UAV-ROS-Packages/assets/53455636/1270df55-0558-4ec1-ae33-525e4e2d2d8d)
![DroneSim1](https://github.com/JoshCaratao/Texas-Aerial-Robotics-UAV-ROS-Packages/assets/53455636/2e9cc587-abd8-4620-9895-bb4f70be4079)
![droneSim4](https://github.com/JoshCaratao/Texas-Aerial-Robotics-UAV-ROS-Packages/assets/53455636/43213459-5d2d-4936-bced-2494a6ccb0f9)

If you have already completed the steps above and want to run offboard control simulations involving aruco marker detection and positioning of the drone precisely over an aruco marker through a simulated downward facing drone camera, you can follow these steps below.

This Simulation environment was developed to better test and validate both our detection scripts and PID Control algorithms for positioning our drone over an aruco marker with precision prior to physical testing.

NOTE: Offboard control refers to control of the UAV/Drone fully autonomously through ROS and the companion computer rather than through a Ground Control Application or remote controller. 

## 1) Catkin Workspace
Ensure your catkin workspace directory has been made as this is where the ROS packages and scripts are stored.

## 2) Install PX4-Autopilot Software and Gazebo Simulator
Download and install the PX4-Autopilo software. This is crucial for running software-in-the-loop (SITL) simulations of our drone and is necesarry for our control package simulations to work correctly. Additionally, Gazebo is the actual physics simulator that allows us to run these simulations.

### Installing PX4 Source Code and development Tool Chain (Gazebo, jmavsim, other simulators)
Use the link below and follow the instructions for "Gazebo, JMAVSim and NuttX (Pixhawk) Targets." 
```
https://docs.px4.io/v1.12/en/dev_setup/dev_env_linux_ubuntu.html#gazebo-jmavsim-and-nuttx-pixhawk-targets
```
  Alternatively, just copy and paste the following commands into your terminal while in your home directory

This will clone the very latest ("main") version of the PX4-Autopilot Repository into the current directory you are in.
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

This will run a script to set up a development environment that includes Gazebo 9 and jMAVSim simulators, and/or the NuttX/Pixhawk toolchain.(Realistically, we only really need Gazebo and the other simulators are not necessarily needed). 
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### Manual Gazebo Download (if above bash script fails)
NOTE: If the above bash script fails to install gazebo, you may have to manually install gazebo instead, with the following commands

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```

Install Gazebo:
```
sudo apt install gazebo9 libgazebo9-dev
```

Restart the computer after completion. You should now have gazebo installed as well as the PX4-Autopilot Software. Try using the following command in your terminal to test and launch gazebo
```
gazebo
```

  You should also see the folder called "PX4-Autopilot" in your home directory.
  #### NOTE 1: you may run into issues with OpenCV later. 
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

#### NOTE 2: You may run into pathing errors for the PX4 packages. To solve this follow the following instructions
Navigate to home directory
```
cd
```
Open .bashrc file in your code editor. This command assumes you are using vscode I believe.
```
code .bashrc
```
Add the following lines at the bottom of the .bashrc file
```
# Catkin workspace setup
source ~/catkin_ws/devel/setup.bash

# Include necessary package paths for PX4-Autopilot SITL Simulations
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
```

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

After the Simulations files are finished downloading, we can move on to copying/pasting them to the proper directories. 

### mavros_posix_sitl.launch file
Copy the "mavros_posix_sitl.launch" file iincluded in my simulation files into following directory
NOTE: There should already be a file called this in the PX4-Autopilot/launch directory. Thus, make sure to delete that first before copying this new version in.
```
PATH_TO_FOLDER/PX4-Autopilot/launch
```
This launch file is responsible for launching MAVROS, PX4 SITL, Gazebo environment, and spawning vehicle. 

### Aruco Models
Copy both the "aruco_6x6_0" and "aruco_6x6_1" folders included in "aruco_models" folder of my simulation files. Paste them into the following directory
```
PATH_TO_FOLDER/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models 
```
These models are necessary for simulating the aruco markers within the gazebo environment.

### Iris Model
Copy the "iris" model folder included in "iris_models" folder of my simulation files. Paste them into the following directory
NOTE: There should already be an "iris" model folder within this directory. You will need to delete this preexisting one before copying this new one in
```
PATH_TO_FOLDER/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```
Unlike the pre-existing iris model, this iris model has been modified to include a downward facing camera on the bottom of the drone that publishes images to a specific image topic through the ROS environment. 

### World file
Copy the "aruco_sim.world" file from the "world_files" folder in my simulation files into the following directory
```
PATH_TO_FOLDER/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
```

## 6) Launching the Simulation
After ensuring all the packages have been successfully built and assuming there are no more errors, you can now run the simulation in gazebo

Navigate to catkin workspace from home directory
```
cd catkin_ws
```
Launch simulation
```
roslaunch controls start_offb_payload_sim.launch
```
This launch file will launch the "mavros_posix_sitl.launch" file mentioned earlier as well as the "Offb_Payload_Sim.cpp" node, which is responsible for the offboard control of this simulated drone, and the "aruco_node_sim.py" node which subscribes to the simulated drone camera topic and performs aruco marker detection.

If the simulation works properly, both the gazebo simulation and a separate window showing the video feed from the simulated drone camera along with detection of the aruco markers. You should be able to observe the drone take off from an initial aruco marker, position itself over a second aruco marker using a feedback Closed-Loop Control algorithm (PID), stay there for a few seconds, and then return to the original aruco marker and land. This is to simulate a drone takeing off, heading to a target (aruco marker), dropping a payload, and then returning to homebase and landing.

NOTE:: If the goals of your autonomous drones are different then what is tested in these simulations. You could easily modify the detection code, control code, and even the simulation environment to better suit your needs. 








  
 







