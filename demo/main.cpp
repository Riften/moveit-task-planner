//
// Created by yongxi on 8/27/21.
//

#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros_data_loader/ros_data_loader.h>

#include <robotflow_log.h>

int main(int argc, const char* argv[]) {
    robotflow::initLogSystem();
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("Demo");
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.allow_undeclared_parameters(true);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("task_planner_demo", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    LOG4CXX_INFO(logger, "Spin Node")
    std::thread([&executor](){
        executor.spin();

    }).detach();

    // Load robot into parameter server
    LOG4CXX_INFO(logger, "Load robot parameters.")
    ROSDataLoader dataloader(node);
    // dataloader.LoadFile("moveit_resources_panda_description", "urdf/panda.urdf", "panda_model");
    dataloader.LoadUrdfSrdf("moveit_resources_panda_description", "urdf/panda.urdf",
                            "moveit_resources_panda_moveit_config", "config/panda.srdf",
                            "panda_model");

    robot_model_loader::RobotModelLoader loader(node, "panda_model", false);
    std::cout << node->get_parameter("panda_model").as_string() << std::endl;

    std::cout << node ->get_parameter("panda_model_planning.joint_limits.virtual_joint/trans_x.max_position")
        << std::endl;
    // Create Robot Model Loader

    // executor.cancel();
    return 0;
}