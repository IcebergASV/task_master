#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/Task.h>
#include <vector>
#include <map>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

class TaskController {
public:
    TaskController(): nh_(""), private_nh_("~"), current_task_num_(0), found_wp_1_(false), found_wp_2_(false)
    {
        // Params
        if (private_nh_.getParam("task_execution_order", task_execution_order_p)) {ROS_INFO_STREAM(TAG << "task_execution_order param set");}
        else{ROS_ERROR_STREAM(TAG << "Failed to load task_execution_order from the parameter server.");}

        if (private_nh_.getParam("navigation_channel_start_point", nav_channel_start_p)) {ROS_INFO_STREAM(TAG << "navigation_channel_start_point param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load navigation_channel_start_point from the parameter server.");}
        
        if (private_nh_.getParam("speed_run_start_point", speed_run_start_p)) {ROS_INFO_STREAM(TAG << "speed_run_start_point param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load speed_run_start_point from the parameter server.");}
        
        if (private_nh_.getParam("docking_start_point", docking_start_p)) {ROS_INFO_STREAM(TAG << "docking_start_point param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load docking_start_point from the parameter server.");}
        
        if (private_nh_.getParam("mag_route_start_point", mag_route_start_p)) {ROS_INFO_STREAM(TAG << "mag_route_start_point param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load mag_route_start_point from the parameter server.");}
        
        if (private_nh_.getParam("return_home_start_point", return_home_start_p)) {ROS_INFO_STREAM(TAG << "return_home_point param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load return_home_start_point from the parameter server.");}

        if (private_nh_.getParam("waypoint_range", waypoint_range_p)) {ROS_INFO_STREAM(TAG << "waypoint_range param set");}
        else {ROS_ERROR_STREAM(TAG << "Failed to load waypoint_range from the parameter server.");}

        // ROS subscribers

        // task_status tells us if we are completed the current task or not
        task_status_sub_ = nh_.subscribe("task_status", 10, &TaskController::taskStatusCallback, this);
        // /mavros/global_position/global gets our current GPS fix
        global_position_ = nh_.subscribe("/mavros/global_position/global", 10, &TaskController::globalPosCallback, this);


        // ROS publishers

        // task_to_execute determines which task should be running
        task_to_execute_ = nh_.advertise<task_master::Task>("task_to_execute", 10);
        // /mavros/setpoint_position/global is the publisher for global start point coordinates
        global_setpoint_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void sortTasks()
    {
        std::vector<std::pair<std::string, int>> sorted_tasks;

        // Exclude items with 0 for the value
        for (const auto& entry : task_execution_order_p) {
            if (entry.second != 0) {
                sorted_tasks.push_back(entry);
            }
        }

        // Sort the tasks by values, least to greatest
        std::sort(sorted_tasks.begin(), sorted_tasks.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        task_master::Task new_task;

        for (const auto& task : sorted_tasks) {
            if (task.first == "navigation_channel")
            {
                new_task.current_task = task_master::Task::NAVIGATION_CHANNEL;
                orderedTasksToExecute_.push_back(new_task);
                ROS_INFO_STREAM(TAG << "Adding nav channel to execute list");
            }
            else if (task.first == "speed_run")
            {
                new_task.current_task = task_master::Task::SPEED_RUN;
                orderedTasksToExecute_.push_back(new_task);
                ROS_INFO_STREAM(TAG << "Adding speed run to execute list");
            }
            else if (task.first == "docking")
            {
                new_task.current_task = task_master::Task::DOCKING;
                orderedTasksToExecute_.push_back(new_task);
                ROS_INFO_STREAM(TAG << "Adding docking to execute list");
            }
            else if (task.first == "mag_route")
            {
                new_task.current_task = task_master::Task::MAG_ROUTE;
                orderedTasksToExecute_.push_back(new_task);
                ROS_INFO_STREAM(TAG << "Adding mag route to execute list");
            }
            else if (task.first == "return_home")
            {
                new_task.current_task = task_master::Task::RETURN_HOME;
                orderedTasksToExecute_.push_back(new_task);
                ROS_INFO_STREAM(TAG << "Adding return home to execute list");
            }
            else 
            {
                ROS_WARN_STREAM(TAG << "invalid task set in task execution order: " << task.first);
            }
                       
        }

        if (orderedTasksToExecute_.size() <= 0){
            ROS_WARN_STREAM(TAG << "No tasks set to execute");
        }
        ROS_DEBUG_STREAM(TAG << "Tasks set: " << orderedTasksToExecute_.size());
    }

    void setTask(int task_num)
    {
        current_task_.current_task = task_num;
        ROS_INFO_STREAM (TAG << "Task set to " << taskNumToString(task_num));
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_status_sub_;
    ros::Subscriber global_position_;
    ros::Publisher task_to_execute_;
    ros::Publisher global_setpoint_;

    task_master::Task current_task_;
    std::vector<task_master::Task> orderedTasksToExecute_;
    int current_task_num_;
    sensor_msgs::NavSatFix global_pose_;
    bool found_wp_1_, found_wp_2_;

    std::string TAG = "TASK_CONTROLLER: ";

    std::map<std::string, int> task_execution_order_p;
    int nav_channel_order_p;
    int speed_run_order_p;
    int docking_order_p;
    int mag_route_order_p;
    int return_home_order_p;

    std::map<std::string, double> nav_channel_start_p;
    std::map<std::string, double> speed_run_start_p;
    std::map<std::string, double> docking_start_p;
    std::map<std::string, double> mag_route_start_p;
    std::map<std::string, double> return_home_start_p;

    double waypoint_range_p;

    // for printing task name for logging
    std::string taskNumToString(uint8_t task_num)
    {
        std::string task_name_string;

        switch(task_num)
        {
            case task_master::Task::TASK_NOT_SET:
            {
                task_name_string = "task_not_set";
                break;
            }
            case task_master::Task::NAVIGATION_CHANNEL:
            {
                task_name_string = "navigation_channel";
                break;
            }
            case task_master::Task::MAG_ROUTE:
            {
                task_name_string = "mag_route";
                break;
            }
            case task_master::Task::DOCKING:
            {
                task_name_string = "docking";
                break;
            }
            case task_master::Task::SPEED_RUN:
            {
                task_name_string = "speed run";
                break;
            }
            case task_master::Task::RETURN_HOME:
            {
                task_name_string = "return home";
                break;
            }
            default:
            {
                ROS_WARN_STREAM(TAG << "Invalid task number for taskNumToString conversion: " << task_num);
                break;
            }

            
        }
        return task_name_string;
    }

    void setNextTask()
    {
        ROS_DEBUG_STREAM(TAG << "Current task num: " << current_task_num_);
        current_task_ = orderedTasksToExecute_[current_task_num_];
        ROS_INFO_STREAM(TAG << "Current task set to execute: " << taskNumToString(current_task_.current_task));
        current_task_num_++;
    }

    bool reachedNextTaskStartPoint()
    {
        double waypoint1_lat, waypoint1_lon, waypoint2_lat, waypoint2_lon;
        if (current_task_.current_task == task_master::Task::SPEED_RUN) {
            waypoint1_lat = speed_run_start_p["lat_1"];
            waypoint1_lon = speed_run_start_p["lon_1"];
            waypoint2_lat = speed_run_start_p["lat_2"];
            waypoint2_lon = speed_run_start_p["lon_2"];
        }
        else if (current_task_.current_task == task_master::Task::DOCKING) {
            waypoint1_lat = docking_start_p["lat_1"];
            waypoint1_lon = docking_start_p["lon_1"];
            waypoint2_lat = docking_start_p["lat_2"];
            waypoint2_lon = docking_start_p["lon_2"];
        }
        else if (current_task_.current_task == task_master::Task::MAG_ROUTE) {
            waypoint1_lat = mag_route_start_p["lat_1"];
            waypoint1_lon = mag_route_start_p["lon_1"];
            waypoint2_lat = mag_route_start_p["lat_2"];
            waypoint2_lon = mag_route_start_p["lon_2"];
        }
        else if (current_task_.current_task == task_master::Task::RETURN_HOME) {
            waypoint1_lat = return_home_start_p["lat_1"];
            waypoint1_lon = return_home_start_p["lon_1"];
            waypoint2_lat = return_home_start_p["lat_2"];
            waypoint2_lon = return_home_start_p["lon_2"];
        }
        else {
            ROS_WARN_STREAM(TAG << "Invalid task set for task start waypoints");
        }

        geographic_msgs::GeoPoseStamped task_start_point;
        if (!found_wp_1_) {
            task_start_point.pose.position.latitude = waypoint1_lat;
            task_start_point.pose.position.longitude = waypoint1_lon;
        }
        else if (!found_wp_2_) {
            task_start_point.pose.position.latitude = waypoint2_lat;
            task_start_point.pose.position.longitude = waypoint2_lon;
        }
        ROS_DEBUG_STREAM(TAG << "Publishing waypoint, lat1 = " << waypoint1_lat << ", lon1 = " << waypoint1_lon << ", lat2 = " << waypoint2_lat << ", lon2 = " << waypoint2_lon);
        global_setpoint_.publish(task_start_point);
        if ((waypoint1_lat>global_pose_.latitude-waypoint_range_p) && (waypoint1_lat<global_pose_.latitude+waypoint_range_p) && (waypoint1_lon>global_pose_.longitude-waypoint_range_p) && (waypoint1_lon<global_pose_.longitude+waypoint_range_p)) {
            found_wp_1_ = true;
        }
        else if ((waypoint2_lat>global_pose_.latitude-waypoint_range_p) && (waypoint2_lat<global_pose_.latitude+waypoint_range_p) && (waypoint2_lon>global_pose_.longitude-waypoint_range_p) && (waypoint2_lon<global_pose_.longitude+waypoint_range_p)) {
            found_wp_2_ = true;
        }

        if (found_wp_1_ && found_wp_2_) {
            ROS_DEBUG_STREAM(TAG << "Found both waypoints");
            return true;
        }
        else {
            return false;
        }
    }

    void taskStatusCallback(const task_master::TaskStatus msg) {
        // ROS_DEBUG_STREAM(TAG << "taskStatusCallback");
        if (orderedTasksToExecute_.size() <= 0)
        {
            ROS_WARN_STREAM(TAG << "No tasks set to execute");
            setTask(task_master::Task::TASK_NOT_SET);
        }
        else
        {
            if (current_task_num_ == 0)
                {
                    setNextTask();
                    ROS_DEBUG_STREAM(TAG << "setNextTask() sucessfull");
                }

            if(msg.task.current_task == current_task_.current_task &&  msg.status == task_master::TaskStatus::COMPLETE)
            {
                if(current_task_num_ < orderedTasksToExecute_.size()-1)
                {
                    ROS_INFO_STREAM(TAG << "Task: " << taskNumToString(msg.task.current_task) << " complete");
                    if (reachedNextTaskStartPoint()) {
                        found_wp_1_ = false;
                        found_wp_2_ = false;
                        setNextTask();
                    }
                }
                else
                {
                    setTask(task_master::Task::TASK_NOT_SET);
                    ROS_INFO_STREAM(TAG << "All set tasks complete!");
                }
            }
            else
            {
                if(msg.task.current_task == current_task_.current_task && msg.status == task_master::TaskStatus::FAILED) //TODO add logic for recovering from a failed task
                {
                    ROS_ERROR_STREAM(TAG << "Task: " << taskNumToString(msg.task.current_task) << " failed, setting task_to_execute to TASK_NOT_SET");
                    setTask(task_master::Task::TASK_NOT_SET);
                }
                else {
                    ROS_DEBUG_STREAM(TAG << "Awaiting " << taskNumToString(msg.task.current_task) << " completion");
                }
            }
        }
 
    }

    void globalPosCallback(const sensor_msgs::NavSatFix msg) {
        global_pose_ = msg;
        // also publish status as this callback always runs
        task_to_execute_.publish(current_task_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_ctrl_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug));
        ros::console::notifyLoggerLevelsChanged();

    TaskController task_controller;

    task_controller.sortTasks();

    task_controller.setTask(task_master::Task::TASK_NOT_SET);
    
    task_controller.spin();

    return 0;
}