//
// Created by yongxi on 8/28/21.
//

#ifndef MOVEIT_TASK_PLANNER_ROS_DATA_LOADER_H
#define MOVEIT_TASK_PLANNER_ROS_DATA_LOADER_H

#include <rclcpp/rclcpp.hpp>
#include <string>

using OnSetParametersCallbackHandle =
        rclcpp::node_interfaces::OnSetParametersCallbackHandle;
using OnParametersSetCallbackType =
        rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;

class ROSDataLoader {
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<OnSetParametersCallbackHandle> set_param_cb_;
public:
    explicit ROSDataLoader(std::shared_ptr<rclcpp::Node> node);
    ~ROSDataLoader();

    /**
     * Read a file from specific ros package and return its contains as string.
     * @param package_name
     * @param relative_path
     * @return File contains.
     */
    static std::string LoadFile(const std::string& package_name, const std::string& relative_path);

    /**
     * Load file into ros node with specific parameter name.
     * @param package_name
     * @param relative_path
     * @param name
     */
    void LoadFile(const std::string& package_name, const std::string& relative_path, const std::string& name);

    /**
     * Load URDF and moveit SRDF file into ros node.
     *
     * @details LoadUrdfSrdf does the same thing as
     * @code
     * LoadFile(urdf_package, urdf_relative_path, name)
     * LoadFile(srdf_package, srdf_relative_path, name + "_semantic")
     * @endcode
     *
     * @param urdf_package
     * @param urdf_relative_path
     * @param srdf_package
     * @param srdf_relative_path
     * @param name
     */
    void LoadUrdfSrdf(const std::string& urdf_package, const std::string& urdf_relative_path,
                      const std::string& srdf_package, const std::string& srdf_relative_path,
                      const std::string& name);

    bool LoadYAML(const std::string& package_name, const std::string& relative_path,
                  const std::string& name);
};

#endif //MOVEIT_TASK_PLANNER_ROS_DATA_LOADER_H
