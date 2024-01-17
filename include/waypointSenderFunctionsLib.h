#ifndef WAYPOINTSENDERFUNCTIONSLIB_H
#define WAYPOINTSENDERFUNCTIONSLIB_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

 struct waypoint {
     float x;
     float y;
     float z;
     float psi;
 };

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);
geometry_msgs::Point get_current_location(nav_msgs::Odometry current_pose);
geometry_msgs::PoseStamped set_heading(float heading, geometry_msgs::PoseStamped waypoint);
geometry_msgs::PoseStamped set_destination(float x, float y, float psi);
int check_waypoint_reached(float tolerance, nav_msgs::Odometry current_pose, geometry_msgs::PoseStamped waypoint);


#endif