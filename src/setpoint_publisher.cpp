#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>

class SetpointPublisher {
public:
    SetpointPublisher() : nh_(""), private_nh_("~"), wp_set_(false), is_reached_(false), count_(0) {
        // Initialize node handle
        //nh_ = ros::NodeHandle("~");
        private_nh_.param<double>("dist", dist_p, 1.0);
        private_nh_.param<bool>("low_freq_cont", low_freq_cont_p, false);
        private_nh_.param<double>("num_setpoints", num_setpoints_p, 1.0);
        private_nh_.param<bool>("low_freq_disc", low_freq_disc_p, false);
        private_nh_.param<bool>("spam", spam_p, false);
        private_nh_.param<bool>("single", single_p, false);
        private_nh_.param<double>("publishing_freq", publishing_freq_p, 1.0);
        // Subscribe to current position
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &SetpointPublisher::poseCallback, this);

        // Subscribe to mavros/mission/reached
        mission_sub_ = nh_.subscribe("/mavros/mission/reached", 10, &SetpointPublisher::missionReachedCallback, this);

        // Subscribe to compass heading
        heading_sub_ = nh_.subscribe("/mavros/imu/data", 10, &SetpointPublisher::headingCallback, this);

        state_sub_ = nh_.subscribe("/mavros/state", 10, &SetpointPublisher::state_cb, this);
        // Publish setpoint
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        // Publish position difference
        position_diff_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/position_diff", 10);

        // Set publishing rate
        publishing_rate_ = nh_.param<double>("publishing_rate", publishing_freq_p); 
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 /publishing_freq_p), &SetpointPublisher::publishTimerCallback, this);
    }

    void sendWP()
    {

        if (pose_set_ && hdg_set_)
        {
            ROS_INFO_STREAM("Sending setpoint: (" <<  setpoint_msg_.pose.position.x << ", " << setpoint_msg_.pose.position.y << ")");
            setpoint_pub_.publish(setpoint_msg_);
        }
    }

    // for low frequency
    void publishTimerCallback(const ros::TimerEvent&) {
        if (low_freq_cont_p || low_freq_disc_p && count_ < num_setpoints_p && current_state_.mode == "GUIDED"){
            if (pose_set_ && hdg_set_ && setpnt_msg_) {
                // Publish setpoint
                sendWP();
                if (low_freq_disc_p) // for controlling discrete low freq
                {
                    count_++;
                }
            }
        }

    }



    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        calculateSetpoint();
        if(spam_p && current_state_.mode == "GUIDED" )
        {
            sendWP();
        }

        pose_set_ = true;
    }

    void missionReachedCallback(const mavros_msgs::WaypointReached msg)
    {
        ROS_INFO_STREAM("Setpoint reached!");
        is_reached_ = true;
        return;
    }

    //get armed state
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
     {
       current_state_ = *msg;
     }


    void headingCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_heading_ = yaw;
        calculateSetpoint();
        hdg_set_ = true;
        if(spam_p && current_state_.mode == "GUIDED")
        {
            sendWP();
        }
    
    }

    void single()
    {
        if(single_p)
        {
            if(current_state_.mode == "GUIDED" && !wp_set_)
            {
                sendWP();
                wp_set_ = true;
            }
        }
    }

    void calculateSetpoint() {
        if (pose_set_ && hdg_set_) {
            // Calculate setpoint
            double setpoint_x = current_pose_.pose.position.x + dist_p * cos(current_heading_);
            double setpoint_y = current_pose_.pose.position.y + dist_p * sin(current_heading_);

            // Publish setpoint
            setpoint_msg_.header.stamp = ros::Time::now();
            setpoint_msg_.header.frame_id = "map"; // or any other frame ID
            setpoint_msg_.pose.position.x = setpoint_x;
            setpoint_msg_.pose.position.y = setpoint_y;
            setpoint_msg_.pose.position.z = current_pose_.pose.position.z; // Maintain current altitude
            setpoint_msg_.pose.orientation = current_pose_.pose.orientation;
            setpnt_msg_ = true;
            //setpoint_pub_.publish(setpoint_msg);

            // Publish position difference
            geometry_msgs::PoseStamped position_diff_msg;
            position_diff_msg.header.stamp = ros::Time::now();
            position_diff_msg.header.frame_id = "map"; // or any other frame ID
            position_diff_msg.pose.position.x = setpoint_x - current_pose_.pose.position.x;
            position_diff_msg.pose.position.y = setpoint_y - current_pose_.pose.position.y;
            position_diff_msg.pose.position.z = 0; // Difference in 2D
            position_diff_pub_.publish(position_diff_msg);
        }
    }


private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber heading_sub_;
    ros::Subscriber mission_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher setpoint_pub_;
    ros::Publisher position_diff_pub_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped setpoint_msg_;
    mavros_msgs::State current_state_;
    ros::Timer publish_timer_;
    double publishing_rate_;
    bool wp_set_;
    bool is_reached_;
    double current_heading_;
    int count_;

    bool pose_set_ = false;
    bool hdg_set_ = false;
    bool setpnt_msg_ = false;

    // Params
    double dist_p;    
    double num_setpoints_p;
    double publishing_freq_p;

    bool low_freq_cont_p;
    bool low_freq_disc_p;
    bool spam_p;
    bool single_p;
    
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_publisher");
    SetpointPublisher setpoint_publisher;

    setpoint_publisher.single();


	while(ros::ok())
	{
        ros::Rate rate(10);
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}