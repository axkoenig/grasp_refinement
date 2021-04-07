#ifndef TASK_POLYTOPE_H
#define TASK_POLYTOPE_H

#include <vector>

#include <tf2/LinearMath/Vector3.h>

class TaskPolytope
{
public:
    int num_task_wrenches = 0;
    std::vector<tf2::Vector3> task_forces;
    std::vector<tf2::Vector3> task_torques;
    void add_task_wrench(tf2::Vector3 task_force, tf2::Vector3 task_torque)
    {
        task_forces.push_back(task_force);
        task_torques.push_back(task_torque);
        num_task_wrenches++;
    }
};

#endif