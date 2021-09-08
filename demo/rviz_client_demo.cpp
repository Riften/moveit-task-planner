//
// Created by yongxi on 2021/9/7.
//

#include <rviz_client/rviz_client.h>
#include <iostream>
#include <robotflow_log.h>

int main(int argc, char* argv[]) {
    robotflow::initLogSystem();
    rclcpp::init(argc, argv);

    RvizClient rvizClient;
    rvizClient.AutoSpin();

    auto params = rvizClient.ListParamNow();
    for(const auto& param : params) {
        std::cout << param << std::endl;
    }

    rvizClient.LoadRobotDescription("moveit_resources_panda_description", "urdf/panda.urdf");

    rclcpp::shutdown();
    return 0;
}