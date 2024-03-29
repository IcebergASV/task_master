#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/Task.h>
#include <vector>
#include <map>
#include <mavros_msgs/SetMode.h>


class TaskController {
public:
    TaskController(): nh_(""), private_nh_("~"), current_task_num_(0)
    {
        // Params
        if (private_nh_.getParam("task_execution_order", task_execution_order_p))
        {
            ROS_INFO_STREAM(TAG << "task_execution_order param set");
        }
        else
        {
            ROS_ERROR_STREAM(TAG << "Failed to load task_execution_order from the parameter server.");
        }

        // ROS subscribers

        // task_status tells us if we are completed the current task or not
        task_status_sub_ = nh_.subscribe("task_status", 10, &TaskController::taskStatusCallback, this);


        // ROS publishers

        // task_to_execute determines which task should be running
        task_to_execute_ = nh_.advertise<task_master::Task>("task_to_execute", 10);

        // ROS services
        state_service_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            task_to_execute_.publish(current_task_);
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
            else 
            {
                ROS_WARN_STREAM(TAG << "invalid task set in task execution order: " << task.first);
            }
                       
        }

        if (orderedTasksToExecute_.size() <= 0){
            ROS_WARN_STREAM(TAG << "No tasks set to execute");
        }
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
    ros::Publisher task_to_execute_;

    task_master::Task current_task_;
    std::vector<task_master::Task> orderedTasksToExecute_;
    int current_task_num_;
    std::string TAG = "TASK_CONTROLLER: ";

    std::map<std::string, int> task_execution_order_p;
    int nav_channel_order_p;
    int speed_run_order_p;
    int docking_order_p;
    int mag_route_order_p;

    ros::ServiceClient state_service_;

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

    void taskStatusCallback(const task_master::TaskStatus msg) {
        ROS_DEBUG_STREAM(TAG << "taskStatusCallback");
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
                }

            if(msg.task.current_task == current_task_.current_task &&  msg.status == task_master::TaskStatus::COMPLETE)
            {
                if(current_task_num_ < orderedTasksToExecute_.size()-1)
                {
                    ROS_INFO_STREAM(TAG << "Task: " << taskNumToString(msg.task.current_task) << " complete");
                    setNextTask();
                }
                else
                {
                    ROS_INFO_STREAM(TAG << "Attempting change to auto mode");
                    mavros_msgs::SetMode mode;
                    mode.request.custom_mode = "AUTO";
                    if (state_service_.call(mode)) {
                        ROS_INFO_STREAM(TAG << "Entered auto mode");
                    }
                    else {
                        ROS_INFO_STREAM(TAG << "Failed to enter auto mode");
                    }
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
            }
        }
 
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_ctrl_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    TaskController task_controller;

    task_controller.sortTasks();

    task_controller.setTask(task_master::Task::TASK_NOT_SET);
    
    task_controller.spin();

    return 0;
}