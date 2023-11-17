#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"   
#include <image_transport/image_transport.h>    
#include <iostream>



int main(int argc, char** argv) {

    //initialize node
    ros::init(argc, argv, "camera_node");

    //create node handler object (needed for ros with c++)
    ros::NodeHandle nh;

    //Create a publisher object 
    //ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imagePub = it.advertise("webCam/Image_raw", 10);

    //creates a CvImage object called img_bridge using cv_bridge
    cv_bridge::CvImage img_bridge;
    img_bridge.encoding = "bgr8";

    sensor_msgs::Image img_msg; //This is the actual msg to be sent

    //Use OpenCV to capture camera feed
    cv::VideoCapture cam;

    ros::Rate connect_rate(1);

    while(ros::ok()){
        cam.open(0); 
        if(!cam.isOpened()){
            ROS_ERROR("Failed to open camera device.");

        }
        else{
            std::cout <<"Camera connected..."<< std::endl;
            break;
        }

        connect_rate.sleep();
    }
       
   
    //set the desired publishing rate (Hz)
    ros::Rate loop_rate(60);  

    //keep capturing image data while ROS master is up and running or if node hasn't been told to shut down
    while(ros::ok()){

        cv::Mat frame;
        cam.read(frame);

        //If no frame is captured, show error and break loop
        if(frame.empty()){
            ROS_ERROR("Failed to capture a frame.");
            break;
        }

        img_bridge.image = frame;
        img_bridge.header.stamp = ros::Time::now();

        //convert to a ROS sensor image and place it in img_msg
        img_bridge.toImageMsg(img_msg);

        //publish ROS Sensor Image
        imagePub.publish(img_msg);

        //Test
        // cv::imshow("C++ WebCam", frame);
        // cv::waitKey(1);
        
        //Although not necessarily needed here, this allowed the ROS framework to process any incoming messages or events.
        //ie, it checks for any incoming messages on the topics I'm publishing or subscribing to and execute the corresponding callback functions.
        ros::spinOnce();

        //this ensures that the loop will run at the desired rate, even if the loop execution time varies.
        loop_rate.sleep();

    }


    cam.release();
    cv::destroyAllWindows();
    
    return 0;


}