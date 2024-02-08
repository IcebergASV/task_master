#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>

#include <task_master/TaskGoalPosition.h>
#include <task_master/TaskStatus.h>
#include <task_master/Waypoint.h>

#include "waypointSenderFunctionsLib.h"

/**
\defgroup control_functions
This module is designed to make high level control programming more simple. 
*/
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped desired_waypoint;
int current_task_to_execute;

task_master::Waypoint goal_pos;

ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber goal_pos_sub;
ros::Subscriber current_task;

std::string TAG = "WP_FILTERSENDER: ";

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
 {
   current_state = *msg;
 }

// set goal pose 
void goal_pose(const task_master::TaskGoalPosition::ConstPtr& msg){

	// Check if msg is from correct task 
	if(current_task_to_execute == msg->task.current_task){
		goal_pos.x = msg->point.x;
		goal_pos.y = msg->point.y;
		goal_pos.z = msg->point.z;
		goal_pos.psi = 0.0;
	}

	desired_waypoint = set_destination(goal_pos.x, goal_pos.y, goal_pos.psi);
	ROS_DEBUG_STREAM(TAG << "About to publish desired coord x: " << desired_waypoint.pose.position.x << ", y:" << desired_waypoint.pose.position.y);
	local_pos_pub.publish(desired_waypoint);

 }

// Get current task to execute 
 void get_current_task(const task_master::Task::ConstPtr& msg){

	current_task_to_execute = msg->current_task;
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
  return current_heading;

}

/*
@returns 0 - connected to fcu 
@returns -1 - failed to connect to drone
*/
int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");

	// Wait for FCU connection
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
		ROS_INFO("Error connecting to FCU");
		return -1;	
	}
}

/**
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4Guided()
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
	}
	else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}

int arm()
{
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
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

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "wp_filter_sender");

	// Set logging level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)){
   		ros::console::notifyLoggerLevelsChanged();
	}
     
	// Initializing publishers and subscribers
	ros::NodeHandle nh;

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	currentPos = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, pose_cb);
	state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
    goal_pos_sub = nh.subscribe<task_master::TaskGoalPosition>("/task_goal_position", 10, goal_pose);
	current_task = nh.subscribe<task_master::Task>("/task_to_execute", 10, get_current_task);

  	// Connect to FCU
	wait4connect();

	// Wait for used to switch to mode GUIDED
	wait4Guided();

	ros::Rate rate(2.0);

	ROS_INFO("Starting mission");

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}