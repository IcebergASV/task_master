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



ros::Subscriber currentPosMavRos;
ros::Publisher currentPosGazebo;

geometry_msgs::PoseStamped current_pose_mavros;
geometry_msgs::PoseStamped current_pose_gazebo;



void get_current_pos(const geometry_msgs::PoseStamped& msg){

    current_pose_mavros = msg;
}

geometry_msgs::PoseStamped convert_2_gazebo(geometry_msgs::PoseStamped currentPos){

    geometry_msgs::PoseStamped convertedPos = currentPos;

    convertedPos.pose.position.x = currentPos.pose.position.y;
    convertedPos.pose.position.y = -1 * currentPos.pose.position.x;

    return convertedPos;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "mavros_2_gazebo_conversion");

    ros::NodeHandle nh;

    currentPosMavRos = nh.subscribe("/mavros/local_position/pose", 10, get_current_pos);
    currentPosGazebo = nh.advertise<geometry_msgs::PoseStamped>("gazebo_pose", 10);

    ros::Rate rate(1);
    while (ros::ok()) {

        current_pose_gazebo = convert_2_gazebo(current_pose_mavros);
        currentPosGazebo.publish(current_pose_gazebo);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}