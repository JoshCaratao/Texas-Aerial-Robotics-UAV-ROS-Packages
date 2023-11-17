#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import sys
import numpy

#Define a global aruco dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#Define global detector Parameters
detectorParams = cv2.aruco.DetectorParameters_create()

#Define global boolean 
arucoDetected = False

#Define Camera intrinsic properties
setCam = False
fx = 1
fy = 1
cx = 0
cy = 0
k1 = 0
k2 = 0
k3 = 0
p1 = 0
p2 = 0
camMatrix = numpy.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
distCoeffs = numpy.array([k1, k2, p1, p2, k3])

#Define Marker dimension
markerLength = 0.05073 #Note this should be in the same unit as camera calibration units



#Call back function for subscrbing and processing webCam data
def aruco_callback(rosImg, pubs):
    #Reference Global variables needed for callback
    global arucoDetected #Only need to do this for global variables we are modifying

    bool_pub = pubs[0]
    tvec_pub = pubs[1]

    #Initialize CV Bridge to convert img formats print("Translation Z: " + str(tvec[2]))
    bridge = CvBridge()

    try:
        #Convert ROS Img Msg to an OpenCV Image
        cvImg = bridge.imgmsg_to_cv2(rosImg, desired_encoding = "bgr8")

        #Temp function. Set cam instrinsics for first loop
        if(setCam == False):
            setCamIntrinsics(cvImg)

        #Call detect function
        markerCorners, markerIds = detectArucoMarkers(cvImg)
        

        #Call Pose Estimation Function
        if(arucoDetected):
            transX, transY, transZ  = estimatePose(markerCorners, markerIds)
            tvec_msg = Point()
            tvec_msg.x = transX
            tvec_msg.y = transY
            tvec_msg.z = transZ
            tvec_pub.publish(tvec_msg)


        #Publish ArucoDetected Data
        bool_pub.publish(arucoDetected)
        
        
        #After processing Display webcam feed with detected aruco marker bounding boxes
        cv2.namedWindow('Aruco WebCam', cv2.WINDOW_NORMAL)
        cv2.imshow("Aruco WebCam", cvImg)
        cv2.waitKey(1)
       
    except Exception as e:
        rospy.logerr(e)



#TEMP FUNCTION TO RUN POS ESTIMATIION BEFORE CAMERA CALIBRATION, automatically sets principle points to center of camera frame
def setCamIntrinsics(cvImg):
    global camMatrix, setCam

    #Get image height and width for cx and cy values
    image_height, image_width, _ = cvImg.shape

    camMatrix[0][2] = 0.5 * image_width
    camMatrix[1][2] = 0.5 * image_height

    setCam = True

    


#Separate Detect aruco markers function to isolate different functions and organize better. 
#This functions deals with aruco detection and drawing boundary boxes and returns markerCorners and Ids
def detectArucoMarkers(cvImg):
    #Reference Global Variables needed for function
    global arucoDetected, markerCorners, markerIds, rejectedCandidates


    #Detect ArucoMarkers
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cvImg, arucoDict, parameters = detectorParams)

    #If markers were detected, draw bounding box
    if (markerIds is not None):
        cv2.aruco.drawDetectedMarkers(cvImg, markerCorners, markerIds)
        arucoDetected = True
    else:
        arucoDetected = False

    #Test prints 
    testString = "Aruco Markers Detected: "
    if(arucoDetected):
        print(testString + str(len(markerIds)))
    else:
        print(testString + "0")    

    return markerCorners, markerIds



#Separate Pose Estimation function to isolate different functions and organize better. 
#This functions deals with Pose estimation of aruco marker with respect to camera frame
def estimatePose(markerCorners, markerIds):
    #Incase there are multiple markers, loop through them to get poses
    for i in range(len(markerIds)):
        rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], markerLength, camMatrix, distCoeffs)

    #print(tvec.ndim)
    transX = tvec[0,0,0]
    transY = tvec[0,0,1]
    transZ = tvec[0,0,2]
    
    #Test prints
    print("Translation X: " + str(transX))
    print("Translation Y: " + str(transY))
    print("Translation Z: " + str(transZ))
    print(" ")

    return transX, transY, transZ


def aruco_node():
    #Initialize Node
    rospy.init_node('aruco_node', anonymous = True)

    #Create an image publisher object for the detected aruco markers image
    image_pub = rospy.Publisher('webcam/Image_aruco', Image, queue_size =10)

    #Create a publisher to publisher boolean if a marker is detected or not
    bool_pub = rospy.Publisher('Aruco/Bool', Bool, queue_size =1)

    #Create a publisher to publish aruco marker translation vector
    tvec_pub = rospy.Publisher('Aruco/tvec', Point, queue_size =1 )

    #Create an image subscriber to subscribe to the webCam topic (make sure to pass in publishers that are used in this callback function)
    rospy.Subscriber("webCam/Image_raw", Image, aruco_callback, (bool_pub, tvec_pub))


    rospy.spin()



if __name__ == '__main__':
    try:
        aruco_node()
    except rospy.ROSInterruptException:
        pass
        