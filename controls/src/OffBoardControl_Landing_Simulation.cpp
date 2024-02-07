#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
geomtry_msgs::PoseStamped current_pose;
bool takeoff = false;

//set arbitrary aruco marker location for simulation
std::vector<int> aruco_pose = {10,10,0}; 

//This is a callback function that is invoked when a new state message is published. Updates knowledge of the current state of the drone
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//This callback function retrieved and updates the local pose of the drone (for PID simulation purposes mainly)
void state_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char**argv){

    //Initialize ROS Node into ROS system and registers name
    ros::init(argc, argv, "OffBoardControl_Landing_Simulation");

    //Create Node Handle (Necessary for communication with ROS system)
    ros::NodeHandle nh;

    //Initialize various subscribers and publishers
    ros::Subscriber drone_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    ros::Subscriber drone_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
    ros::Publisher local_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2hz
    ros::Rate rate(20.0)


    //This loop ensures that the rest of the node doesn't run until ROS has connected to the flight control unit. 
    //After moving on from simulation implementation, this should also include detection of aruco marker
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Initialize the messages for publishing
    geometry_msgs::PoseStamped pose_commands;
    geometry_msgs::Twist vel_commands;

    pose_commands.pose.position.x = 0;
    pose_commands.pose.position.y = 0;
    pose_commands.pose.position.z = 0;

    //These probably depend on the orientation of the pixhawk flight controller on the drone frame
    vel_commands.linear.x = 0;  //Forward velocity  
    vel_commands.linear.y = 0;  //left/right velocity 
    vel_commands.linear.z = 0;  //up/down velocity 
    vel_commands.angular.x = 0; //Roll Rate
    vel_commands.angular.y = 0; //Pitch Rate
    vel_commands.angular.z = 0; //Yaw Rate

    //Initialize PID Loop Variables
    double last_x_error;
    double last_y_error;
    double last_z_error;

    double total_x_error;
    double total_y_error;
    double total_z_error; 
    



    //Initialize service request commands
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;

    //Set a timer to time requests for arming 
    ros::Time last_request = ros::Time::now();

    //Start main loop
    while(ros::ok()){

        ros::spinOnce();

        //Check if drone is NOT in offboard mode
        if(current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)){
            //If not in offboard mode, request offboard mode
            if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent){
                ROS_INFO("Offboard Enabled")
            }
            last_request = ros::Time::now();
        }

        //Ater enabling offboard, check if drone is armed. If not armed, request to arm drone
        else{
            if(!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
            last_request = ros::Time::now();
        }

        //Check if drone has taken off already.(different methods to do this)
        if(takeoff == false || current_pose.pose.position.z == 0){
            //takeoff code
            pose_commands.pose.position.x = 0;
            pose_commands.pose.position.y = 0;
            pose_commands.pose.position.z = 7;

            local_position_pub.publish(pose_commands);
        }

        //If drone has already taken off, begin PID Loop
        else{

            //PID Code

        }




    }


    //start z PID loop? or switch modes for landing?

    return 0;
}

std::vector<double>PID(double error, double last_error) {
    

}