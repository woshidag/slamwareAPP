#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#include <rpos/robot_platforms/slamware_core_platform.h>

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::types;
using namespace rpos::core;

class Task {
public:
    

    enum taskType
    {
        moveType = 0,
        goHomeType,
    };
    enum taskType task_type_;

    enum taskStatus
    {
        start = 0,
        running,
        end,
    };
    enum taskStatus task_status_;

    Task(Task::taskType type, rpos::core::Pose& start_pose, rpos::core::Pose& target_pose);

private:
    int task_id_;
    long start_time_;
    long end_time_;

    rpos::core::Pose start_pose_;
    rpos::core::Pose target_pose_;

    bool is_arride_;
    std::string reason;//the reason why robot was not arrivde

    int try_count_;
};

class TaskManager {
public:
    TaskManager();

    void addTask(boost::shared_ptr<Task> task);
    void update();
private:
    std::vector<boost::shared_ptr<Task>> tasks_;    
};

#endif