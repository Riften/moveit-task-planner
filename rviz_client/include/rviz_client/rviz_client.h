//
// Created by yongxi on 2021/9/8.
//

#ifndef MOVEIT_TASK_PLANNER_RVIZ_CLIENT_H
#define MOVEIT_TASK_PLANNER_RVIZ_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <log4cxx/logger.h>
#include <chrono>

class RvizClient {
private:
    /// Default time out is 300 ms
    std::chrono::milliseconds time_out_ = std::chrono::milliseconds(300);
    log4cxx::LoggerPtr logger_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_params_cli_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_cli_;
    void init_client();
public:

    /**
     * Create an RvizClient as a new ros2 node.
     * @param node_name Name of ros2 node to be created. Default is rviz_client
     */
    explicit RvizClient(const std::string& node_name = "rviz_client");

    /**
     * Create an RvizClient running on the given node.
     * @param node
     */
    explicit RvizClient(rclcpp::Node::SharedPtr node);

    rclcpp::Node::SharedPtr Node() {
        return node_;
    }

    /**
     * Spin the ros2 node in a separating thread.
     */
    void AutoSpin();

    std::shared_future<rcl_interfaces::srv::ListParameters::Response::SharedPtr>
    ListParam();

    /**
     * List all parameters.
     *
     * @todo Raise an error when time out so that we can throw it to python.
     * @return An array of parameter names.
     */
    std::vector<std::string> ListParamNow();

    /**
     *
     * @param param
     * @return
     */
    std::shared_future<rcl_interfaces::srv::SetParameters::Response::SharedPtr>
    SetParam(const rclcpp::Parameter& param);

    bool LoadRobotDescription(const std::string& package,
                              const std::string& relative_path);
};

#endif //MOVEIT_TASK_PLANNER_RVIZ_CLIENT_H
