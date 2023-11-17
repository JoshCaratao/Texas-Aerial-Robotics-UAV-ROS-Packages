#!/usr/bin/env python

import rospy
#from std_msgs.msg import Header
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

 
def camera_node():

    #Initialize Node
    rospy.init_node('camera_node', anonymous = True)

    #Create an image publisher object
    image_pub = rospy.Publisher('webCam/Image_raw', Image, queue_size = 10)

    #Use open cv to get camera footage (0 is default cam)
    webCam = cv2.VideoCapture(0)

    #initialize cv_bridge
    bridge = CvBridge()

    #Set the publishing rate 
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        ret, webCamFrame = webCam.read()

        if(ret):
            #convert to a ROS Image msg using bridge and publish
            image_msg = bridge.cv2_to_imgmsg(webCamFrame, encoding = "bgr8")
            image_pub.publish(image_msg)

            #Display image feed for testing
            cv2.imshow("Python WebCam", webCamFrame)
            cv2.waitKey(1)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
        
