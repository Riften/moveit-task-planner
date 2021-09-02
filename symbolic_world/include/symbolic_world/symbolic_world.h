//
// Created by yongxi on 9/2/21.
//

#ifndef MOVEIT_TASK_PLANNER_SYMBOLIC_WORLD_H
#define MOVEIT_TASK_PLANNER_SYMBOLIC_WORLD_H

#include <moveit/collision_detection/world.h>

class SymbolicWorld {
public:
    SymbolicWorld(collision_detection::WorldPtr world);
private:
    collision_detection::WorldPtr moveit_world_;
    void init();
};

#endif //MOVEIT_TASK_PLANNER_SYMBOLIC_WORLD_H
