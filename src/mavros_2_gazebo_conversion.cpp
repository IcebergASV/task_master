#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <ros/ros.h>

ros::Subscriber currentPosMavRos;
ros::Publisher currentPosGazebo;

geometry_msgs::PoseStamped current_pose_mavros;
geometry_msgs::PoseStamped current_pose_gazebo;

std::string TAG = "MRtoGZCONVERSION: ";

void get_current_pos(const geometry_msgs::PoseStamped& msg){

    current_pose_mavros = msg;
}

geometry_msgs::Point convertPosition(geometry_msgs::Point mavros_position)
{
    geometry_msgs::Point gazebo_position;

    gazebo_position.x = mavros_position.y;
    gazebo_position.y = -mavros_position.x;

    ROS_DEBUG_STREAM(TAG << "Gazebo position x: " << gazebo_position.x << ", y: " << gazebo_position.y);
    return gazebo_position;
}

geometry_msgs::Quaternion convertOrientation(const geometry_msgs::Quaternion q)
{
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    // convert yaw to gazebo yaw
    double gz_yaw = yaw - (M_PI/2);
    if (gz_yaw < 0)
    {
        gz_yaw = gz_yaw + (2*M_PI);
    }

    ROS_DEBUG_STREAM(TAG << "Gazebo Heading " << gz_yaw);

    // Convert back to quaternion with updated yaw
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(gz_yaw * 0.5);
    double sy = sin(gz_yaw * 0.5);

    geometry_msgs::Quaternion gz_q;
    gz_q.w = cr * cp * cy + sr * sp * sy;
    gz_q.x = sr * cp * cy - cr * sp * sy;
    gz_q.y = cr * sp * cy + sr * cp * sy;
    gz_q.z = cr * cp * sy - sr * sp * cy;

    return gz_q;
}

geometry_msgs::PoseStamped convertMavrosToGazeboPose(geometry_msgs::PoseStamped mavros_pose){

    geometry_msgs::PoseStamped gazebo_pose = mavros_pose;

    gazebo_pose.pose.position = convertPosition(mavros_pose.pose.position);
    gazebo_pose.pose.orientation = convertOrientation(mavros_pose.pose.orientation);

    return gazebo_pose;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mavros_2_gazebo_conversion");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    ros::NodeHandle nh;

    currentPosMavRos = nh.subscribe("/mavros/local_position/pose", 10, get_current_pos);
    currentPosGazebo = nh.advertise<geometry_msgs::PoseStamped>("gazebo_pose", 10);

    ros::Rate rate(10);
    while (ros::ok()) {

        current_pose_gazebo = convertMavrosToGazeboPose(current_pose_mavros);
        currentPosGazebo.publish(current_pose_gazebo);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}