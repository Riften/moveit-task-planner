/**
 * The implementation of BulletClient refers to
 * bullet3/examples/RobotSimulator/RobotSimulatorMain.cpp
 */

#ifndef MOVEIT_TASK_PLANNER_BULLET_CLIENT_H
#define MOVEIT_TASK_PLANNER_BULLET_CLIENT_H

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
class BulletClient {
public:
    BulletClient();
private:
    b3RobotSimulatorClientAPI* client_;
};

#endif //MOVEIT_TASK_PLANNER_BULLET_CLIENT_H
