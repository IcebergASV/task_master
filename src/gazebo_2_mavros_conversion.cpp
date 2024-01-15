#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <ros/ros.h>

ros::Subscriber desired_pos_gazebo_sub;
ros::Publisher desired_pos_mavros_pub;

geometry_msgs::PoseStamped desired_pose_mavros;
geometry_msgs::PoseStamped desired_pose_gazebo;

std::string TAG = "GZtoMR_CONVERSION: ";

void getDesiredGZPoseCallback(const geometry_msgs::PoseStamped& msg){

    desired_pose_gazebo = msg;
}

geometry_msgs::Point convertPosition(geometry_msgs::Point gazebo_position)
{
    geometry_msgs::Point mavros_position;

    mavros_position.x = -gazebo_position.y;
    mavros_position.y = gazebo_position.x;

    ROS_DEBUG_STREAM(TAG << "Mavros position x: " << mavros_position.x << ", y: " << mavros_position.y);
    return mavros_position;
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
    double mavros_yaw = yaw + (M_PI/2);
    if (mavros_yaw > 2*M_PI)
    {
        mavros_yaw = mavros_yaw - (2*M_PI);
    }

    ROS_DEBUG_STREAM(TAG << "Mavros Heading " << mavros_yaw);

    // Convert back to quaternion with updated yaw
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(mavros_yaw * 0.5);
    double sy = sin(mavros_yaw * 0.5);

    geometry_msgs::Quaternion mavros_q;
    mavros_q.w = cr * cp * cy + sr * sp * sy;
    mavros_q.x = sr * cp * cy - cr * sp * sy;
    mavros_q.y = cr * sp * cy + sr * cp * sy;
    mavros_q.z = cr * cp * sy - sr * sp * cy;

    return mavros_q;
}

geometry_msgs::PoseStamped convertGazeboToMavrosPose(geometry_msgs::PoseStamped gazebo_pose){

    geometry_msgs::PoseStamped mavros_pose = gazebo_pose;

    mavros_pose.pose.position = convertPosition(gazebo_pose.pose.position);
    mavros_pose.pose.orientation = convertOrientation(gazebo_pose.pose.orientation);

    return mavros_pose;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "gazebo_2_mavros_conversion");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
    ros::NodeHandle nh;

    desired_pos_gazebo_sub = nh.subscribe("/gazebo_conversion/mavros/local_position/pose", 10, getDesiredGZPoseCallback);
    desired_pos_mavros_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10);


    ros::Rate rate(10);
    while (ros::ok()) {

        desired_pose_mavros = convertGazeboToMavrosPose(desired_pose_gazebo);
        desired_pos_mavros_pub.publish(desired_pose_mavros);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}