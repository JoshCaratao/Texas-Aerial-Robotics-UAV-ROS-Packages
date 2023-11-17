In progress computer_vision ROS package for the drone. This package deals with accessing and publishing camera images. Also deals with computer vision to detect aruco markers and estimate pose of Aruco marker with respect to the camera frame.

11/5/23 UPDATE: the two main working files are aruco_node_py.py and camera_node_py.py. These were reworked in python due to issues in C++ that we still have to work on.

The Camera node interfaces with a connected webcam, and publishes image data while the Aruco node subscribes to the webcam topic and process the image data with OpenCV to detect Aruco Markers.

11/7/23 UPDATE: Redid the C++ Camera Node. Now, the two main files are "camera_node.cpp" and "aruco_node.py" They work together the same as before, but one is just now in C++ again. Also added in Aruco marker position estimation w/respect to the camera frame. I have yet to add proper camera calibration however.
