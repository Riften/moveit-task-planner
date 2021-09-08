//
// Created by yongxi on 2021/9/8.
//

#include <rviz_client/rviz_client.h>

// Note that we only use the static functions of ros data loader
#include <ros_data_loader/ros_data_loader.h>

#include <utility>

static const std::string rviz_set_parameters_srv = "/rviz/set_parameters";
static const std::string rviz_list_parameters_srv = "/rviz/list_parameters";
static const std::string rviz_param_robot_description = "robot_description";

void RvizClient::AutoSpin() {
    std::thread([this](){
        LOG4CXX_INFO(this->logger_, "Node Spin Starts")
        rclcpp::spin(this->node_);
        LOG4CXX_INFO(this->logger_, "Node Spin Ends")
    }).detach();
}

void RvizClient::init_client() {
    list_params_cli_ = node_->create_client<rcl_interfaces::srv::ListParameters>(rviz_list_parameters_srv);
    set_params_cli_ = node_->create_client<rcl_interfaces::srv::SetParameters>(rviz_set_parameters_srv);
    while(!(list_params_cli_->wait_for_service(std::chrono::milliseconds(300))
            && set_params_cli_->wait_for_service(std::chrono::milliseconds(300)))) {
        if(!rclcpp::ok()) {
            LOG4CXX_FATAL(logger_, "rclcpp interrupted")
        } else {
            LOG4CXX_WARN(logger_, "service not ready")
        }
    }
    LOG4CXX_INFO(logger_, "service ready")
}

RvizClient::RvizClient(const std::string& node_name) {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    logger_ = log4cxx::Logger::getLogger("RvizClient");
    init_client();
}

RvizClient::RvizClient(rclcpp::Node::SharedPtr node) {
    node_ = std::move(node);
    logger_ = log4cxx::Logger::getLogger("RvizClient");
    init_client();
}

std::shared_future<rcl_interfaces::srv::ListParameters::Response::SharedPtr>
RvizClient::ListParam() {
    auto req = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    // req->prefixes.push_back("");
    req->depth = 1;
    return list_params_cli_->async_send_request(req);
}

std::vector<std::string> RvizClient::ListParamNow() {
    auto res_future = ListParam();
    if (res_future.wait_for(time_out_) != std::future_status::ready) {
        LOG4CXX_ERROR(logger_, "List parameters timeout")
        return {};
    }
    return res_future.get()->result.names;
}

std::shared_future<rcl_interfaces::srv::SetParameters::Response::SharedPtr>
RvizClient::SetParam(const rclcpp::Parameter &param) {
    // Build service request
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param.to_parameter_msg());

    // Send request
    return set_params_cli_->async_send_request(request);
}

bool RvizClient::LoadRobotDescription(const std::string &package, const std::string &relative_path) {
    std::string contains = ROSDataLoader::LoadFile(package, relative_path);

    // Build parameter
    rclcpp::Parameter parameter(rviz_param_robot_description,
                      contains);
    auto res_future = SetParam(parameter);
    if(res_future.wait_for(time_out_) != std::future_status::ready) {
        LOG4CXX_ERROR(logger_, "Set robot description timeout")
        return false;
    }

    std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response = res_future.get();
    if(response->results.empty()) {
        LOG4CXX_ERROR(logger_, "Rviz server return no response for set robot_description")
        return false;
    }

    if(!response->results.begin()->successful) {
        LOG4CXX_ERROR(logger_, "Set robot_description fails because " \
                                        << response->results.begin()->reason.c_str())
        return false;
    }

    return true;
}