//
// Created by yongxi on 8/30/21.
//

#ifndef MOVEIT_TASK_PLANNER_BULLET_APP_H
#define MOVEIT_TASK_PLANNER_BULLET_APP_H

#include <CommonInterfaces/CommonGraphicsAppInterface.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <CommonInterfaces/CommonGUIHelperInterface.h>


/**
 * A simple bullet gui application.
 *
 * The implementation reference `bullet3/examples/StandaloneMain/main_opengl_single_example.cpp`
 */
class BulletApp {
public:
    explicit BulletApp(int window_width = 1024, int window_height = 768);
    ~BulletApp();

    /**
     * Update current frame manually.
     */
    void update();

    btDiscreteDynamicsWorld* getWorld();

private:
    CommonGraphicsApp* graphics_app_;

    /**
         * Initialize Bullet Dynamics World and maintain it in dynamics_world_.
         *
         * @details The implementation references
         * bullet3/examples/BasicDemo/BasicExample.cpp: createEmptyDynamicsWorld()
         */
    void initDynamicsWorld();
    btDiscreteDynamicsWorld* dynamics_world_;
    GUIHelperInterface* gui_helper;
};

#endif //MOVEIT_TASK_PLANNER_BULLET_APP_H
