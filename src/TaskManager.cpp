#include "TaskManager.h"

Task::Task(Task::taskType type, rpos::core::Pose& start_pose, rpos::core::Pose& target_pose):task_status_(taskStatus::start),
    task_id_(0), start_time_(0),end_time_(0),is_arride_(false),try_count_(0)
{
    this->task_type_ = type;
    this->start_pose_ = start_pose;
    this->target_pose_ = target_pose;
}

TaskManager::TaskManager()
{

}

void TaskManager::addTask(boost::shared_ptr<Task> task)
{
    this->tasks_.push_back(task);
}

void TaskManager::update()
{

}