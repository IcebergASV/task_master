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

#include <task_master/TaskGoalPosition.h>

struct Quaternion 
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternion(EulerAngles e) 
{
    // roll (x), pitch (y), yaw (z), angles are in radians
    // Abbreviations for the various angular functions

    double cr = cos(e.roll * 0.5);
    double sr = sin(e.roll * 0.5);
    double cp = cos(e.pitch * 0.5);
    double sp = sin(e.pitch * 0.5);
    double cy = cos(e.yaw * 0.5);
    double sy = sin(e.yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

ros::Subscriber currentPosMavRos;
ros::Publisher currentPosGazebo;

geometry_msgs::PoseStamped current_pose_mavros;
geometry_msgs::PoseStamped current_pose_gazebo;

std::string TAG = "MAVROS 2 GZ: ";



void get_current_pos(const geometry_msgs::PoseStamped& msg){

    current_pose_mavros = msg;
}

geometry_msgs::PoseStamped convert_2_gazebo(geometry_msgs::PoseStamped currentPos){

    geometry_msgs::PoseStamped convertedPos = currentPos;

    // Convert Position
    convertedPos.pose.position.x = currentPos.pose.position.y;
    convertedPos.pose.position.y = -1 * currentPos.pose.position.x;

    // Convert Orientation - shift 90 degrees counter clockwise

    Quaternion orientation_q;
    orientation_q.w = convertedPos.pose.orientation.w;
    orientation_q.x = convertedPos.pose.orientation.x;
    orientation_q.y = convertedPos.pose.orientation.y;
    orientation_q.z = convertedPos.pose.orientation.z;

    // convert quaternion to euler angles
    EulerAngles orientation_e = ToEulerAngles(orientation_q);

    ROS_DEBUG_STREAM(TAG << "Mavros Heading: " << orientation_e.yaw*(180/M_PI));

    // shift yaw by 90 degrees
    orientation_e.yaw = orientation_e.yaw - (M_PI/2);

    // convert back to quaternion        
    orientation_q = ToQuaternion(orientation_e);

    EulerAngles converted_heading = ToEulerAngles(orientation_q);
    ROS_DEBUG_STREAM(TAG << "Gazebo Heading: " << converted_heading.yaw*(180/M_PI));

    // Update pose with shifted orientation
    convertedPos.pose.orientation.w = orientation_q.w;
    convertedPos.pose.orientation.x = orientation_q.x;
    convertedPos.pose.orientation.y = orientation_q.y;
    convertedPos.pose.orientation.z = orientation_q.z;

    return convertedPos;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "mavros_2_gazebo_conversion");
	
    // Set logging level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
   		ros::console::notifyLoggerLevelsChanged();
	}

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