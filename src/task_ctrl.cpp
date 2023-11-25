#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/Task.h>

class TaskController {
public:
    TaskController(): nh_(""), private_nh_("~")
    {
        // ROS subscribers

        // task_status tells us if we are completed the current task or not
        task_status_sub_ = nh_.subscribe("task_status", 10, &TaskController::taskStatusCallback, this);


        // ROS publishers

        // task_to_execute determines which task should be running
        task_to_execute_ = nh_.advertise<task_master::TaskStatus>("task_to_execute", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            task_to_execute_.publish(current_status_);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void setDefaultStatus(int new_task) {
        current_status_.status = task_master::TaskStatus::NOT_STARTED;
        current_status_.task.current_task = new_task;
        ROS_INFO("Set default task and task status.");
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_status_sub_;
    ros::Publisher task_to_execute_;

    task_master::TaskStatus current_status_;

    void taskStatusCallback(const task_master::TaskStatus msg) {
        current_status_ = msg;
        if(msg.status == task_master::TaskStatus::FAILED || msg.status == task_master::TaskStatus::COMPLETE) {
            // when task fails or completes, we publish task_not_set
            task_master::TaskStatus taskStatus;
            taskStatus.task.current_task = task_master::Task::TASK_NOT_SET;
            taskStatus.status = msg.status;
            if (taskStatus.status == task_master::TaskStatus::FAILED) {
                ROS_INFO("Task failed.");
            }
            else {
                ROS_INFO("Task completed.");
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_ctrl_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    TaskController task_controller;

    task_controller.setDefaultStatus(task_master::Task::NAVIGATION_CHANNEL);
    
    task_controller.spin();

    return 0;
}