/**#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <fstream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_sender");
    ros::NodeHandle nh;

    ros::ServiceClient push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

    // Wait for the service to become available
    push_client.waitForExistence();

    // Load waypoints from params.yaml
    std::vector<ros::param::Parameter> waypoint_params;
    nh.getParam("waypoints", waypoint_params);

    mavros_msgs::WaypointPush push_msg;
    for (const auto& param : waypoint_params) {
        mavros_msgs::Waypoint waypoint;
        nh.getParam(param.getName(), waypoint);
        push_msg.request.waypoints.push_back(waypoint);
    }

    // Call the service to push the waypoints
    if (push_client.call(push_msg)) {
        ROS_INFO("Waypoints pushed successfully!");
    } else {
        ROS_ERROR("Failed to push waypoints");
        return 1;
    }

    return 0;
}**/

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_sender");
    ros::NodeHandle nh;

    ros::ServiceClient push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

    // Wait for the service to become available
    push_client.waitForExistence();

    mavros_msgs::WaypointPush push_msg;
    mavros_msgs::Waypoint waypoint;

    // Define your waypoints here
    waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
    waypoint.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint.is_current = false;
    waypoint.autocontinue = true;
    waypoint.param1 = 0.0; // Hold time in seconds
    waypoint.param2 = 5.0; // Acceptance radius in meters
    waypoint.param3 = 0.0; // Pass radius in meters
    waypoint.param4 = 0.0; // Desired yaw angle
    waypoint.x_lat = 27.37425520; // Latitude
    waypoint.y_long = -82.45287340; // Longitude
    waypoint.z_alt = 10.0; // Altitude in meters

    // Add the waypoint to the message
    push_msg.request.waypoints.push_back(waypoint);

    // Call the service to push the waypoints
    if (push_client.call(push_msg)) {
        ROS_INFO("Waypoint pushed successfully!");
    } else {
        ROS_ERROR("Failed to push waypoint");
        return 1;
    }

    return 0;
}
