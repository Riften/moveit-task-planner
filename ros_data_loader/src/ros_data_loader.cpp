//
// Created by yongxi on 8/28/21.
//

#include "ros_data_loader/ros_data_loader.h"

#include <utility>
#include <boost/filesystem.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "log4cxx/logger.h"
#include "robotflow_log.h"
#include "yaml-cpp/yaml.h"

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
            LOG4CXX_DEBUG(logger, "Parameter set: " << param.get_name() << "  Type:"<< param.get_type())
        }

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    };

    //set_param_cb_ = node_->add_on_set_parameters_callback(func);
}

ROSDataLoader::~ROSDataLoader() {
    //node_->remove_on_set_parameters_callback(set_param_cb_.get());
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
    node_->declare_parameter(name);
    node_->set_parameter(param);
}

void ROSDataLoader::LoadUrdfSrdf(const std::string &urdf_package, const std::string &urdf_relative_path,
                                 const std::string &srdf_package, const std::string &srdf_relative_path,
                                 const std::string &name) {
    std::string srdf_param_name = name + "_semantic";
    LoadFile(urdf_package, urdf_relative_path, name);
    LoadFile(srdf_package, srdf_relative_path, srdf_param_name);
}

/**
 * YAML cpp does not provide any method to judge the type of scalar.
 * But ros parameter need the type info when set a parameter.
 * In that case, we need several exception check to do that.
 * This is not efficient, but works.
 *
 * @param name Name of ros parameter.
 * @param yaml_node Yaml scalar node
 * @return Constructed ros parameter.
 */
static inline rclcpp::Parameter construct_param_from_yaml_scalar(const std::string& name,
                                                          const YAML::Node& yaml_node) {
    try {
        bool value = yaml_node.as<bool>();
        auto param = rclcpp::Parameter(name, value);
        return param;
    } catch (const YAML::BadConversion& e) {
        // do nothing
    }

    try {
        int value = yaml_node.as<int>();
        auto param = rclcpp::Parameter(name, value);
        return param;
    } catch (const YAML::BadConversion& e) {
        // do nothing
    }

    try {
        double value = yaml_node.as<double>();
        auto param = rclcpp::Parameter(name, value);
        return param;
    } catch (const YAML::BadConversion& e) {
        // do nothing
    }

    try {
        std::string value = yaml_node.as<std::string>();
        auto param = rclcpp::Parameter(name, value);
        return param;
    } catch (const YAML::BadConversion& e) {
        // do nothing
    }

    LOG4CXX_ERROR(logger, "Can not determine the type of parameter " << name)
    return {};
}

static void access_yaml_node_recursive(YAML::Node& yaml_node, const std::string& prefix,
                                       std::function<void(rclcpp::Parameter& param)> callback) {

    switch(yaml_node.Type()) {
        case YAML::NodeType::Scalar:
            if(prefix.empty()) {
                LOG4CXX_WARN(logger, "yaml node without prefix")
            } else {
                // auto param = rclcpp::Parameter(prefix, yaml_node.as<std::string>());
                rclcpp::Parameter param = construct_param_from_yaml_scalar(prefix, yaml_node);
                callback(param);
            }
            break;
        case YAML::NodeType::Sequence:
            {
                // Iterate sequence and build parameter value
                std::vector<std::string> parameter_value;
                for(auto sequence_node : yaml_node) {
                    if(!sequence_node.IsScalar()) {
                        LOG4CXX_ERROR(logger, "access a yaml nested sequence " << \
                        prefix << "." << sequence_node.as<std::string>())
                    }
                    parameter_value.push_back(sequence_node.as<std::string>());
                }

                // Call callback
                auto param = rclcpp::Parameter(prefix, parameter_value);
                callback(param);
            }
            break;
        case YAML::NodeType::Map:
            {
                // append node name behind prefix
                std::string new_prefix;
                for(auto map_node : yaml_node) {
                    if(prefix.empty()) {
                        new_prefix = map_node.first.as<std::string>();
                    } else {
                        new_prefix = prefix + "." + map_node.first.as<std::string>();
                    }

                    // access node recursively
                    access_yaml_node_recursive(map_node.second, new_prefix, callback);
                }
            }
            break;
        default:
            LOG4CXX_ERROR(logger, "invalid yaml node type: " << yaml_node.Type())
    }

}

bool ROSDataLoader::LoadYAML(const std::string &package_name, const std::string &relative_path,
                             const std::string &name) {
    /// Tutorial of libyaml can be found from https://www.wpsoftware.net/andrew/pages/libyaml.html

    // Find File
    LOG4CXX_DEBUG(logger, "Load yaml file " << package_name << ": " << relative_path)
    boost::filesystem::path dir_path = ament_index_cpp::get_package_share_directory(package_name);
    LOG4CXX_DEBUG(logger, "Package " << package_name << " path: " << dir_path)
    boost::filesystem::path file_path =
            dir_path / boost::filesystem::path(relative_path);


    // Parse file
    YAML::Node yaml = YAML::LoadFile(file_path.string());
    if (yaml.IsNull()) {
        LOG4CXX_ERROR(logger, "Open file failed: " << file_path)
        return false;
    }

    // Load to node parameter server
    access_yaml_node_recursive(yaml, name, [this](rclcpp::Parameter& param)->void{
        if (this->node_->has_parameter(param.get_name())) {
            LOG4CXX_WARN(logger, "Parameter " << param.get_name() << "already exists and would be override")
        } else {
            this->node_->declare_parameter(param.get_name());
        }
        this->node_->set_parameter(param);
    });

    return true;
}