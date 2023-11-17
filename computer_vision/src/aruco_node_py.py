#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import sys

#Define a global aruco dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#Define global detector Parameters
detectorParams = cv2.aruco.DetectorParameters_create()

#Define global boolean 
arucoDetected = False


def aruco_callback(rosImg, bool_pub):

    #Reference Global variables needed for aruco detection
    global arucoDict
    global detectorParams

    #Initialize CV Bridge to convert img formats
    bridge = CvBridge()

    try:
        #Convert ROS Img Msg to an OpenCV Image
        cvImg = bridge.imgmsg_to_cv2(rosImg, desired_encoding = "bgr8")

        #Detect ArucoMarkers
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cvImg, arucoDict, parameters = detectorParams)

        #If markers were detected, draw bounding box
        if (markerIds is not None):
            cv2.aruco.drawDetectedMarkers(cvImg, markerCorners, markerIds)
            arucoDetected = True
        else:
            arucoDetected = False

        #Publish Data
        bool_pub.publish(arucoDetected)
        print(arucoDetected)

        #display webcam feed with detected aruco marker bounding boxes
        cv2.imshow("Web Cam", cvImg)
        cv2.waitKey(1)
       
    except Exception as e:
        rospy.logerr(e)


def aruco_node_py():
    #Initialize Node
    rospy.init_node('aruco_node', anonymous = True)

    #Create an image publisher object for the detected aruco markers image
    image_pub = rospy.Publisher('webcam/Image_aruco', Image, queue_size = 10)

    #Create a publisher to publisher boolean if a marker is detected or not
    bool_pub = rospy.Publisher('Aruco/Bool', Bool, queue_size = 1)

    #Create an image subscriber to subscribe to the webCam topic (make sure to pass in publishers that are used in this callback function)
    rospy.Subscriber("webCam/Image_raw", Image, aruco_callback, bool_pub)


    rospy.spin()



if __name__ == '__main__':
    try:
        aruco_node_py()
    except rospy.ROSInterruptException:
        pass
        