//
// Created by yongxi on 8/27/21.
//

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros_data_loader/moveit_data_loader.h>
#include <moveit/planning_interface/planning_interface.h>

#include <robotflow_log.h>
#include <boost/scoped_ptr.hpp>

static const std::string ompl_plugin_name = "ompl_interface/OMPLPlanner";
static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("Demo");

planning_interface::PlannerManagerPtr LoadOmplPlanner(rclcpp::Node::SharedPtr & ros_node,
                     const moveit::core::RobotModelConstPtr& robot_model,
                     const std::string& parameter_namespace,
                     const std::string& planner_plugin_name = "ompl_interface/OMPLPlanner") {
    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        LOG4CXX_ERROR(logger, "Can not create class loader for planning_interface::PlannerManager")
        return {};
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, ros_node, parameter_namespace))
            LOG4CXX_ERROR(logger, "Can not initialize planner_interface")
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
            ss << cls << " ";
        LOG4CXX_ERROR(logger, "Exception while loading planner '" << planner_plugin_name << "': " << ex.what()
            << '\n' << "Available plugins: " << ss.str())
    }

    return planner_instance;
}

int main(int argc, const char* argv[]) {
    robotflow::initLogSystem();

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);
    // node_options.allow_undeclared_parameters(true);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("task_planner_demo", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    LOG4CXX_INFO(logger, "Spin Node")
    std::thread([&executor](){
        executor.spin();

    }).detach();

    // Load robot into parameter server
    LOG4CXX_INFO(logger, "Load robot parameters.")

    MoveitDataLoader dataloader(node, "panda");
    dataloader.LoadURDF("moveit_resources_panda_description", "urdf/panda.urdf");
    dataloader.LoadSRDF("moveit_resources_panda_moveit_config", "config/panda.srdf");
    dataloader.LoadJointLimits("moveit_resources_panda_moveit_config",
                               "config/joint_limits.yaml");
    dataloader.LoadYAML("moveit_resources_panda_moveit_config",
                        "config/ompl_planning.yaml",
                        "ompl_configuration");


    //std::cout << node->get_parameter("panda_model") << std::endl;
    robot_model_loader::RobotModelLoader loader(node, "panda", false);

    LOG4CXX_INFO(logger, "Create ompl planner")
    planning_interface::PlannerManagerPtr planner = LoadOmplPlanner(node,
                                                                    loader.getModel(),
                                                                    "ompl_configuration");
    if(planner == nullptr) {
        LOG4CXX_FATAL(logger, "Empty planner")
    }

    /*
    LOG4CXX_INFO(logger, "Test list parameter")
    auto query_res = node->list_parameters(std::vector<std::string>{}, 10);
    for (unsigned long i = 0; i< query_res.names.size(); ++i) {
        LOG4CXX_DEBUG(logger, "Parameter list: " << query_res.names[i])
    }
     */

    // std::cout << node ->get_parameter("panda_model_planning.joint_limits.virtual_joint/trans_x.max_position")
    //     << std::endl;
    // Create Robot Model Loader

    // executor.cancel();
    rclcpp::shutdown();
    return 0;
}