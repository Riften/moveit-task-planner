//
// Created by yongxi on 8/29/21.
//

#ifndef MOVEIT_TASK_PLANNER_MOVEIT_DATA_LOADER_H
#define MOVEIT_TASK_PLANNER_MOVEIT_DATA_LOADER_H

#include "ros_data_loader/ros_data_loader.h"

class MoveitDataLoader : public ROSDataLoader {
public:
    MoveitDataLoader(std::shared_ptr<rclcpp::Node> node, const std::string& robot_description);
    void LoadURDF(const std::string& package_name, const std::string& relative_path);
    void LoadSRDF(const std::string& package_name, const std::string& relative_path);
    void LoadJointLimits(const std::string& package_name, const std::string& relative_package);
private:
    // std::string robot_description_;
    std::string urdf_namespace_;
    std::string srdf_namespace_;
    std::string joint_limits_namespace_;
};

#endif //MOVEIT_TASK_PLANNER_MOVEIT_DATA_LOADER_H
