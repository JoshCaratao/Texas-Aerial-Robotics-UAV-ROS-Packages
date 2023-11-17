#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"   
#include <image_transport/image_transport.h>    


void arucoCallback(const sensor_msgs::ImageConstPtr& rosFrame){
    
    
    std::cout << "started arucoCallback" << std::endl;
    cv::Mat testImg = cv::imread("/home/ubuntu/OpenCVTutorials/OpenCV_Tutorials/Resources/test.png");
    cv::imshow("Webcam", testImg);
    cv::waitKey(0);

    //Convert ROS image message to OpenCV format
    cv_bridge::CvImagePtr cvFrame = cv_bridge::toCvCopy(rosFrame, "bgr8");
    // cv_bridge::CvImagePtr cvFrame;
    // cvFrame = cv_bridge::toCvCopy(rosFrame, sensor_msgs::image_encodings::BGR8);
    std::cout <<"Started openCV" << std::endl;
    cv:: Mat frame = cvFrame->image;
    
    // cv::imshow("webcam", frame);
    // cv::waitKey(1); 

    // try{  

       
    //     //Convert ROS image message to OpenCV format
    //     //cv_bridge::CvImagePtr cvFrame = cv_bridge::toCvCopy(rosFrame, "bgr8");
    //     cv_bridge::CvImagePtr cvFrame;
    //     cvFrame = cv_bridge::toCvCopy(rosFrame, sensor_msgs::image_encodings::BGR8);
    //     cv:: Mat frame = cvFrame->image;
    //     std::cout <<"Started openCV" << std::endl;

        
        
        

    //     cv::imshow("webcam", frame);
    //     cv::waitKey(1); 

              

    //     std::cout << "hello" << std::endl;
       
    
    // }

    // catch(cv_bridge::Exception& e){
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    // }
    

}

int main(int argc, char** argv) {

    //initialize node
    ros::init(argc, argv, "aruco_node2");

    //create node handler object (needed for ros with c++)
    ros::NodeHandle nh;

    //Create a publisher object 
    //ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("camera/image",1,arucoCallback);
    //creates a CvImage object called cvFrame which is our openCV image format
    cv_bridge::CvImage cvFrame;
    cvFrame.encoding = "bgr8";

    //Use OpenCV to capture camera feed
    

    //set the desired publishing rate (Hz)
    ros::Rate loop_rate(20);  
 
    //keep capturing image data while ROS master is up and running or if node hasn't been told to shut down
    


    while(ros::ok()){
        std::cout << "Before Spin" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
    
    return 0;


}