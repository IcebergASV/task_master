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
            startTask(task_master::Task::NAVIGATION_CHANNEL);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void startTask(int new_task) {
        while (current_status_.status != task_master::TaskStatus::IN_PROGRESS) {
            ROS_INFO_STREAM("Publishing initial task, task " << new_task);
            // allows us to set what task we want to run
            task_master::TaskStatus task;
            task.task.current_task = new_task;
            task.status = task_master::TaskStatus::NOT_STARTED;
            task_to_execute_.publish(task);
        }
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
            task_master::Task task;
            task.current_task = task_master::Task::TASK_NOT_SET;
            ROS_INFO("Publishing task to execute.");
            task_to_execute_.publish(task);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_ctrl_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    TaskController task_controller;

    //task_controller.startTask(1);    
    //task_controller.startTask(task_master::Task::NAVIGATION_CHANNEL);    
    task_controller.spin();

    return 0;
}