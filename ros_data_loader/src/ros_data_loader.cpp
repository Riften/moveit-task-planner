//
// Created by yongxi on 8/28/21.
//

#include "ros_data_loader/ros_data_loader.h"

#include <utility>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "log4cxx/logger.h"
#include "robotflow_log.h"

using OnSetParametersCallbackHandle =
        rclcpp::node_interfaces::OnSetParametersCallbackHandle;
using OnParametersSetCallbackType =
        rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;

static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("ROSDataLogger");
// static OnSetParametersCallbackHandle set_param_callback_ = ;


static inline bool file_exists(const std::string& file_path) {
    std::ifstream f(file_path.c_str());
    return f.good();
}

ROSDataLoader::ROSDataLoader(std::shared_ptr<rclcpp::Node> node)
    :node_(std::move(node)) {
    OnParametersSetCallbackType func = [](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult{
        // Note that logger is a static global variable which
        // can be used in lambda without capture.

        for(const auto& param: params) {
            LOG4CXX_DEBUG(logger, "Parameter set callback: " << param.get_name())
        }

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    };

    set_param_cb_ = node_->add_on_set_parameters_callback(func);
}

ROSDataLoader::~ROSDataLoader() {
    node_->remove_on_set_parameters_callback(set_param_cb_.get());
}

std::string ROSDataLoader::LoadFile(const std::string &package_name, const std::string &relative_path) {
    LOG4CXX_DEBUG(logger, "Load file " << package_name << ": " << relative_path)
    boost::filesystem::path dir_path = ament_index_cpp::get_package_share_directory(package_name);
    LOG4CXX_DEBUG(logger, "Package " << package_name << " path: " << dir_path)
    boost::filesystem::path file_path =
            dir_path / boost::filesystem::path(relative_path);
    //if(file_exists(file_path.string())) {
    std::ifstream input(file_path.string());
    if(!input.good()) {
        LOG4CXX_ERROR(logger, "Can not open file " << file_path)
        return "";
    }
    std::ostringstream sstr;
    sstr << input.rdbuf();
    input.close();
    return sstr.str();
}

void ROSDataLoader::LoadFile(const std::string &package_name, const std::string &relative_path,
                             const std::string &name) {
    std::string contains = LoadFile(package_name, relative_path);
    if(contains.empty()) {
        LOG4CXX_WARN(logger, "File " << package_name << ": " << relative_path << " is empty." )
    }
    auto param = rclcpp::Parameter(name, contains);
    LOG4CXX_DEBUG(logger, "Set parameter " << name)

    /**
     * MoveIt would not declare_parameter before set it.
     * In that case, we need to config the node to automatically declare parameters.
     * So the explicit parameter declaration here is removed.
     * However, MoveIt may change their strategy in the future as ROS2 parameters
     * need to be declared before setting.
     */
    // node_->declare_parameter(name);
    node_->set_parameter(param);
}

void ROSDataLoader::LoadUrdfSrdf(const std::string &urdf_package, const std::string &urdf_relative_path,
                                 const std::string &srdf_package, const std::string &srdf_relative_path,
                                 const std::string &name) {
    std::string srdf_param_name = name + "_semantic";
    LoadFile(urdf_package, urdf_relative_path, name);
    LoadFile(srdf_package, srdf_relative_path, srdf_param_name);
}