//
// Created by yongxi on 8/29/21.
//

#include "ros_data_loader/moveit_data_loader.h"
#include "log4cxx/logger.h"

static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("MoveitDataLoader");

MoveitDataLoader::MoveitDataLoader(std::shared_ptr<rclcpp::Node> node, const std::string &robot_description): ROSDataLoader(node) {
    // robot_description_ = robot_description;
    urdf_namespace_ = robot_description;
    srdf_namespace_ = robot_description + "_semantic";
    joint_limits_namespace_ = robot_description + "_planning";
}

void MoveitDataLoader::LoadURDF(const std::string &package_name, const std::string &relative_path) {
    LoadFile(package_name, relative_path, urdf_namespace_);
}

void MoveitDataLoader::LoadSRDF(const std::string &package_name, const std::string &relative_path) {
    LoadFile(package_name, relative_path, srdf_namespace_);
}

void MoveitDataLoader::LoadJointLimits(const std::string &package_name, const std::string &relative_package) {
    LoadYAML(package_name, relative_package, joint_limits_namespace_);
}
