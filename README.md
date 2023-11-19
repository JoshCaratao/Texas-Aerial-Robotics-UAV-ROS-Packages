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

## **Software Installations**
### 1. Have a proper installation of Ubunuto 18.04 (Bionic)

  This could be done either through a dedicated linux machine, dual booting a laptop/PC, or using a USB Bootdrive with Ubuntu 18.04

### 2. Install and build ROS Melodic

  Once you are sure your system meets the minimum requirements for ROS Melodic (Ubuntu 18.04), you can continue with the installation of ROS Melodic
  Follow the instructions in the following link: https://wiki.ros.org/melodic/Installation/Ubuntu

*Note* if you are unable to install the Desktop-Full version, you can install a different version and install other packages/dependencies as you need them

### 3. Installation of OpenCV

  For our purposes, I recommend installing OpenCV for C++, which will take longer, but you can also install OpenCV for Python if you want

#### **OpenCV with C++**

  The installation can be completed by following the link below. Highly highly recommend following the "install from source" option. 
    https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

*IMPORTANT NOTES BEFORE INSTALLING*

If you do not need the contrib_module files then on step 2, do not clone the “opencv_contrib repository, also ensure to exclude “-D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \” when running the cmake command:  

If you are installing this on the Jetson TX2 already, Make –j4 instead of –j8 this is due to the TX2 only having 4 cores. 

If you wish to install an older version of OPENCV use git checkout as mentioned. Version 4.7 has worked so far on with Ubuntu 18.04, ROS MelodicI**

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
