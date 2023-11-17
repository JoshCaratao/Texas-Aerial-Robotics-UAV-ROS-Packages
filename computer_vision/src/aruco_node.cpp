#include "ros/ros.h"
#include "sensor_msgs/Image.h"
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>     
#include  <image_transport/image_transport.h>
#include "std_msgs/Int8.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
              

int main(int argc, char** argv) {

   //Initialize Node and nodehandler
    ros::init(argc, argv, "aruco_node");
    ros::NodeHandle nh;

    //Create Image Transport object and create publisher object using ImageTransport class
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imageSub = it.advertise("webcam/image",1);

    //Subscribe using the ros general publisher
    //ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("camera/image", 1, arucoCallback);

    //Create a ROS publisher for if an aruco marker is detected
    //ros::Publisher detected_pub = nh.advertise<std_msgs::Bool>("arucoDetected", 10);
    //std_msgs::Bool arucoDetected;


    //Create a ROS publisher for the aruco marker ID
    //ros::Publisher Id_pub = nh.advertise<std_msgs::Int8>("arucoID", 10);
    //std_msgs::Int8 arucoID;

    //Initalize webcam
    cv::VideoCapture webCam;
    webCam.open(0);
    
    if(!webCam.isOpened()){
       ROS_ERROR("Failed to open camera device.");
       return -1; 
    }

    std::cout<< "WEBCAM INITIALIZED" << std::endl;


    //initalize Window for testing
    cv::namedWindow("DetectedAruco", cv::WINDOW_AUTOSIZE);

   //Initialize necessary variables for aruco marker detection an image output
    cv::Mat inputImg;
    cv::Mat detectedImg;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    //create detector parameters
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    std::cout<< "DETECTOR PARAMETERS CREATED" << std::endl;

    //create aruco dictionary
    cv::aruco::Dictionary arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::cout<< "ARUCO DICTIONARY CREATED CREATED" << std::endl;

    //create an aruco detector object, with detector parameters
    cv::aruco::ArucoDetector detector(arucoDict, detectorParams);
    std::cout<< "ARUCO DETECTOR CREATED" << std::endl;


    ros::Rate loop_rate(30);

    while(ros::ok()){

        std::cout<< "ENTERED LOOP" << std::endl;

        webCam.read(inputImg);
        //webCam >> inputImg;
        std::cout<< "WEBCAM READ" << std::endl;

        detectedImg = inputImg.clone();
        std::cout<< "DETECTED IMAGE CREATED" << std::endl;

        detector.detectMarkers(inputImg, markerCorners, markerIds, rejectedCandidates);
        //cv::aruco::detectMarkers(inputImg, arucoDict, markerCorners, markerIds, detectorParams, rejectedCandidates);

        if(markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(detectedImg, markerCorners, markerIds);
        }

        cv::imshow("DetectedAruco", detectedImg);
        cv::waitKey(1);


        

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
   


}