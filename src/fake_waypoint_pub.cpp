#include <ros/ros.h>
#include <task_master/TaskGoalPosition.h>
#include <cmath>


void fake_waypoint() {
    ros::NodeHandle nh("");
    ros::NodeHandle private_nh_("~");

  

    ros::Publisher pub = nh.advertise<task_master::TaskGoalPosition>("goal_position", 1);
    ros::Rate rate(1);
    task_master::TaskGoalPosition msg;

    // Message
    msg.x = 15;
    msg.y = 0; 
    msg.z = 0;

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_waypoint_pub");
    try {
        fake_waypoint();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred: " << e.what());
    }
    return 0;
}