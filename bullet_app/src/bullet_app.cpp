//
// Created by yongxi on 8/30/21.
//
#include "bullet_app/bullet_app.h"
#include <OpenGLWindow/SimpleOpenGL3App.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <ExampleBrowser/OpenGLGuiHelper.h>
#include "log4cxx/logger.h"

log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("BulletApp");
log4cxx::LoggerPtr bullet3Logger = log4cxx::Logger::getLogger("Bullet3");

BulletApp::BulletApp(int window_width, int window_height)
    : dynamics_world_(nullptr) {
    b3SetCustomWarningMessageFunc([](const char* msg){
        LOG4CXX_WARN(bullet3Logger, msg)
    });
    b3SetCustomPrintfFunc([](const char* msg){
        LOG4CXX_INFO(bullet3Logger, msg)
    });
    b3SetCustomErrorMessageFunc([](const char* msg){
        LOG4CXX_ERROR(bullet3Logger, msg)
    });

    // Create Graphics App
    const std::string title = "Demo OpenGL App for RobotFlow";
    LOG4CXX_INFO(logger, "Create opengl gui app with title " << title)

    graphics_app_ = new SimpleOpenGL3App(title.c_str(), window_width, window_height);
    //graphics_app_->setBackgroundColor(1,1,1);
    graphics_app_->setUpAxis(2); // Use Z axis as up axis

    // Create canvas
    // GL3TexLoader* myTexLoader = new GL3TexLoader;

    // Set camera
    graphics_app_->m_renderer->getActiveCamera()->setCameraTargetPosition(0, 0, 0);
    graphics_app_->m_renderer->getActiveCamera()->setCameraDistance(13);
    graphics_app_->m_renderer->getActiveCamera()->setCameraPitch(0);

    // Create GUIHelper
    gui_helper = new OpenGLGuiHelper(graphics_app_, false);
    // gui_helper->syncPhysicsToGraphics()

    if(graphics_app_->m_renderer) {
        LOG4CXX_INFO(logger, "app render initialized")
    } else {
        LOG4CXX_INFO(logger, "app render has not been initialized")
    }

    initDynamicsWorld();

}

BulletApp::~BulletApp() {
    // Note that delete a null pointer is ok.
    delete dynamics_world_;
    delete graphics_app_;
}

void BulletApp::initDynamicsWorld() {
    auto collision_cfg = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    auto collision_dispatcher = new btCollisionDispatcher(collision_cfg);

    // Broad phase collision detector.
    // It can filter the object pairs and avoid further computation for some pairs.
    auto broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    auto* sol = new btSequentialImpulseConstraintSolver;


    dynamics_world_ = new btDiscreteDynamicsWorld(collision_dispatcher, broadphase, sol, collision_cfg);

    //m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    // Use z axis as up
    dynamics_world_->setGravity(btVector3(0, 0, -10));
}

void BulletApp::update() {
    // gui_helper->syncPhysicsToGraphics(dynamics_world_);
    gui_helper->autogenerateGraphicsObjects(dynamics_world_);
    gui_helper->render(dynamics_world_);
    graphics_app_->swapBuffer();
}

btDiscreteDynamicsWorld *BulletApp::getWorld() {
    return dynamics_world_;
}