#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdlib>
#include "controls/PID.h"

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool aruco_detected = false;
bool takeoff = false;
bool payload_deployed = false;

//set arbitrary aruco marker location for simulation
std::vector<int> aruco_pose = {10,10,2.5}; 

//Set position of home base
std::vector<int> home_base = {0,0,2.5};

//set target position coordinate
std::vector<int> target_pos = {0,0,0};


//This is a callback function that is invoked when a new state message is published. Updates knowledge of the current state of the drone
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//This callback function retrieved and updates the local pose of the drone (for PID simulation purposes mainly)
void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

//This callback function retrieved and updates the local pose of the drone (for PID simulation purposes mainly)
void aruco_detected_callback(const std_msgs::Bool::ConstPtr& msg){
    aruco_detected = msg->data;
}

int main(int argc, char**argv){

    //Initialize ROS Node into ROS system and registers name
    ros::init(argc, argv, "OffBoardControl_Landing_Simulation");

    //Create Node Handle (Necessary for communication with ROS system)
    ros::NodeHandle nh;


    //Initialize various subscribers and publishers
    ros::Subscriber drone_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    ros::Subscriber drone_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
    ros::Publisher local_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 30);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //ros::Subscriber aruco_detector_sub = nh.subscribe<std_msgs::Bool>("Aruco/Bool", 10, aruco_detected_callback);

    //the setpoint publishing rate MUST be faster than 2hz
    ros::Rate rate(20.0);


    //This loop ensures that the rest of the node doesn't run until ROS has connected to the flight control unit. 
    //After moving on from simulation implementation, this should also include detection of aruco marker
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Initialize the messages for publishing
    geometry_msgs::PoseStamped pose_commands;
    geometry_msgs::TwistStamped vel_commands;

    pose_commands.pose.position.x = 0;
    pose_commands.pose.position.y = 0;
    pose_commands.pose.position.z = 2;

   
    //These probably depend on the orientation of the pixhawk flight controller on the drone frame
    vel_commands.twist.linear.x = 0;  //Forward velocity  
    vel_commands.twist.linear.y = 0;  //left/right velocity 
    vel_commands.twist.linear.z = 0;  //up/down velocity 
    vel_commands.twist.angular.x = 0; //Roll Rate
    vel_commands.twist.angular.y = 0; //Pitch OFFBOARD

    PID pidX(0.2, 0.05, 0.005);
    PID pidY(0.2, 0.05, 0.005);
    PID pidZ(0.1, 0.75, 0);

    std::string mode = current_state.mode;
    //ROS_INFO("CURRENT MODE: %s",mode.c_str());

    //Initialize service request commands
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;

    // while(!aruco_detected){
    //     //runs this loop until aruco is detected
    // }

    //Set a timer to time requests for arming 
    ros::Time last_request = ros::Time::now();

    //Set timers for deploying payload sim
    ros::Time last_loop_time = ros::Time::now();
    double convergeTime = 0;

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_position_pub.publish(pose_commands);
        ros::spinOnce();
        rate.sleep();
    }

    //Start main loop
    while(ros::ok()){
        
        //checks for incoming messages (subscriptions) and invokes callback function if there are
        ros::spinOnce();

        //Could add a section here to check drone local position and send out to ros info
        
        //Check if drone is armed and if not, request to arm the drone
        if(!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){
            if(arming_client.call(arm_command) && arm_command.response.success){
                ROS_INFO("Vehicle armed");
            }

            last_request = ros::Time::now();
        }

        else{ //Once armed, request to switch to offboard mode
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                //If not in offboard mode, request offboard mode
                if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent){
                    ROS_INFO("Offboard Enabled");
                }
                last_request = ros::Time::now();
            }
        }
    
        ros::spinOnce();

        //Check if drone has taken off already.(different methods to do this)
        double z_pos = current_pose.pose.position.z;
        double x_pos = current_pose.pose.position.x;
        double y_pos = current_pose.pose.position.y;
        ROS_INFO("CURRENT X: %f",x_pos);
        ROS_INFO("CURRENT Y: %f",y_pos);
        ROS_INFO("CURRENT Z: %f",z_pos);
    
        if(current_pose.pose.position.z < 2 && !takeoff){
            //takeoff code
            local_position_pub.publish(pose_commands);
        }
        else{

            if(!takeoff){
                takeoff = true;
            }

            if(!payload_deployed){
                target_pos = aruco_pose;
            }
            else{
                target_pos = home_base;
            }

            //Calculate Errors
            double errorX = target_pos[0] - current_pose.pose.position.x;
            double errorY = target_pos[1] - current_pose.pose.position.y;
            double errorZ = target_pos[2] - current_pose.pose.position.z;
    
            //SET VELOCITIES FROM PID
            vel_commands.twist.linear.x = pidX.calculate(errorX, ros::Time::now().toSec());
            vel_commands.twist.linear.y = pidY.calculate(errorY, ros::Time::now().toSec());
            vel_commands.twist.linear.z = pidZ.calculate(errorZ, ros::Time::now().toSec());

            velocity_pub.publish(vel_commands);

            ROS_INFO("xVel: %f", vel_commands.twist.linear.x);
            ROS_INFO("yVel: %f", vel_commands.twist.linear.y);
            ROS_INFO("zVel: %f", vel_commands.twist.linear.z);

            if(std::abs(errorX) < 0.2 && std::abs(errorY) < 0.2 && !payload_deployed){
                convergeTime += ros::Time::now().toSec() - last_loop_time.toSec();
                if(convergeTime >= 3){
                    payload_deployed = true;
                }
            }


        }

        //this is for the converge time
        last_loop_time = ros::Time::now();


        //This ensures that our main loop runs at specified frequency we set above
        rate.sleep();
        mode = current_state.mode;
        ROS_INFO("CURRENT MODE: %s",mode.c_str());

    }

    return 0;
}

