#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/Task.h>
#include <vector>

class TaskController {
public:
    TaskController(): nh_(""), private_nh_("~")
    {
        // Params
        private_nh_.param<double>("navigation_channel_order", nav_channel_order_p, 1);
        private_nh_.param<double>("speed_run_order", speed_run_order_p, 0);
        private_nh_.param<double>("docking_order", docking_order_p, 0);
        private_nh_.param<double>("mag_route_order", mag_route_order_p, 0);

        // ROS subscribers

        // task_status tells us if we are completed the current task or not
        task_status_sub_ = nh_.subscribe("task_status", 10, &TaskController::taskStatusCallback, this);


        // ROS publishers

        // task_to_execute determines which task should be running
        task_to_execute_ = nh_.advertise<task_master::Task>("task_to_execute", 10);

        // initialize task order vector
        task_execution_order_.assign(4, task_master::Task::TASK_NOT_SET);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            task_to_execute_.publish(current_task_);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_status_sub_;
    ros::Publisher task_to_execute_;

    task_master::Task current_task_;
    std::vector<task_master::Task> task_execution_order_;
    int current_task_num_;
    std::string TAG = "TASK_CONTROLLER: ";

    int nav_channel_order_p;
    int speed_run_order_p;
    int docking_order_p;
    int mag_route_order_p;

    //void setDefaultStatus(int new_task) {
    //    current_task_.current_task = new_task;
    //    ROS_INFO_STREAM(TAG << "Set default task and task status.");
    //}

    void setTask(int task_number)
    {
        current_task_.current_task = task_execution_order_[task_number];
        

    }

    void setTask(task_master::Task task)
    {
        current_task_.current_task = task;
        ROS_INFO_STREAM (TAG << "Task set to " << task);

    }

    void mapTasks()
    {
        std::vector<int> task_orders = {nav_channel_order_p, speed_run_order_p, docking_order_p, mag_route_order_p};

        // Ensure task orders are valid 
        for (int i : mag_route_order_p)
        {
            if (i > 4 || i < 0)
            {
                ROS_WARN_STREAM(TAG << "Invalid task order number: " << i);
            }
        }

        // Set the task order
        if(nav_channel_order_p != 0) {task_execution_order_[nav_channel_order_p-1] = task_master::Task::NAVIGATION_CHANNEL;}
        if(speed_run_order_p != 0) {task_execution_order_[speed_run_order_p-1] = task_master::Task::SPEED_RUN;};
        if (docking_order_p != 0 ) {task_execution_order_[docking_order_p-1] = task_master::Task::DOCKING;}
        if (mag_route_order_p != 0) {task_execution_order_[mag_route_order_p-1] = task_master::Task::MAG_ROUTE;}

        // Remove any unset values in the task execution order array
        task_execution_order_.erase(std::remove_if(task_execution_order_.begin(), task_execution_order_.end(), [](task_master::Task task) { return task == task_master::Task::TASK_NOT_SET; }), task_execution_order_.end());

        ROS_INFO_STREAM(TAG << "Task execution order: " << task_execution_order_ );
        return;
    }


    void taskStatusCallback(const task_master::TaskStatus msg) {
        ROS_DEBUG_STREAM(TAG << "taskStatusCallback");
        //current_status_ = msg;
        if(msg.status == task_master::TaskStatus::FAILED || msg.status == task_master::TaskStatus::COMPLETE) {
            // when task fails or completes, we publish task_not_set
            current_task_.current_task = task_master::Task::TASK_NOT_SET;
            if (msg.status == task_master::TaskStatus::FAILED) {
                ROS_INFO_STREAM( TAG << "Task failed.");
            }
            else {
                ROS_INFO_STREAM(TAG << "Task completed.");
            }
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_ctrl_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    TaskController task_controller;

    task_controller.mapTasks();

    task_controller.settask(task_master::Task::TASK_NOT_SET);
    
    task_controller.spin();

    return 0;
}