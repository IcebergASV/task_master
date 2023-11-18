#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>

#include <task_master/Waypoint.h>
#include <task_master/TaskGoalPosition.h>

#include "waypointSenderFunctionsLib.h"

/**
\defgroup control_functions
This module is designed to make high level control programming more simple. 
*/
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::Pose correction_vector;
geometry_msgs::Point local_offset_pose;
geometry_msgs::PoseStamped desired_waypoint;

float current_heading;
float local_offset;
float correction_heading = 0;
float local_desired_heading; 

waypoint goal_pos;


ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber goal_pos_sub;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
 {
   current_state = *msg;
 }

void pose(const task_master::TaskGoalPosition& msg){
    goal_pos.x = msg.goal_pose.x;
    goal_pos.y = msg.goal_pose.y;
    goal_pos.z = msg.goal_pose.z;
    goal_pos.psi = 0.0;
 }

float pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  enu_2_local(current_pose);
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  float current_heading = psi*(180/M_PI);
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  return current_heading;

}

float get_current_heading()
 {
 	return current_heading;
 }

/**
\ingroup control_functions
Wait for connect is a function that will hold the program until communication with the FCU is established.
@returns 0 - connected to fcu 
@returns -1 - failed to connect to drone
*/
int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}
/**
\ingroup control_functions
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4start()
{
	ROS_INFO("Waiting for user to set mode to GUIDED");
	while(ros::ok() && current_state.mode != "GUIDED")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state.mode == "GUIDED")
	{
		ROS_INFO("Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}
/**
\ingroup control_functions
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
@returns 0 - frame initialized
*/
int initialize_local_frame()
{
	//set the orientation of the local reference frame
	ROS_INFO("Initializing local coordinate system");
	local_offset = 0;
	for (int i = 1; i <= 30; i++) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		

		float q0 = current_pose.pose.pose.orientation.w;
		float q1 = current_pose.pose.pose.orientation.x;
		float q2 = current_pose.pose.pose.orientation.y;
		float q3 = current_pose.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset += psi*(180/M_PI);

		local_offset_pose.x = local_offset_pose.x + current_pose.pose.pose.position.x;
		local_offset_pose.y = local_offset_pose.y + current_pose.pose.pose.position.y;
		local_offset_pose.z = local_offset_pose.z + current_pose.pose.pose.position.z;
		// ROS_INFO("current heading%d: %f", i, local_offset_g/i);
	}
	local_offset_pose.x = local_offset_pose.x/30;
	local_offset_pose.y = local_offset_pose.y/30;
	local_offset_pose.z = local_offset_pose.z/30;
	local_offset /= 30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X' axis is facing: %f", local_offset);
	return 0;
}

int arm()
{
	//intitialize first waypoint of mission
	set_destination(0,0,0,0,correction_heading, local_desired_heading, correction_vector, local_offset_pose, desired_waypoint);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(desired_waypoint);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(desired_waypoint);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return 0;
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}
}


/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(ros::NodeHandle controlnodehandle)
{
	local_pos_pub = controlnodehandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	currentPos = controlnodehandle.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, pose_cb);
	state_sub = controlnodehandle.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	arming_client = controlnodehandle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = controlnodehandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    //goal_pos_sub =controlnodehandle.subscribe<task_master::TaskGoalPosition>("/hi",10,pose);
    
}


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "wp_filter_sender");
	ros::NodeHandle nh;
    goal_pos_sub = nh.subscribe("/goal_position",10,pose);
    ros::spin();

	
	//initialize control publisher/subscribers
	init_publisher_subscriber(nh);

  	// wait for FCU connection
	wait4connect();

    
	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
    initialize_local_frame();

	// arm boat 
	//arm();

   


	//specify some waypoints 
	// std::vector<waypoint> waypointList;
	//waypoint nextWayPoint; 
	//nextWayPoint.x = goal_pos.x;
	//nextWayPoint.y = goal_pos.y;
	//nextWayPoint.z = goal_pos.z;
	//nextWayPoint.psi = 0;

	ros::Rate rate(2.0);

	ROS_INFO("Starting mission");
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		desired_waypoint = set_destination(goal_pos.x, goal_pos.y, goal_pos.z, goal_pos.psi, correction_heading, local_desired_heading, correction_vector, local_offset_pose, desired_waypoint);
		local_pos_pub.publish(desired_waypoint);
        ROS_DEBUG("current pose x %F y %f z %f", (current_pose.pose.pose.position.y), -1 *(current_pose.pose.pose.position.x), (current_pose.pose.pose.position.z));
		
	}
	return 0;
}