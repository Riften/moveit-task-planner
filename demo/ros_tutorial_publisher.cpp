//
// Created by yongxi on 2021/9/6.
//
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <unistd.h>
class TutPublisher : public rclcpp::Node {
public:
    TutPublisher(): rclcpp::Node("tut_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);
    }
    void publish() {
        std_msgs::msg::String message;
        message.data = "Hello this is publisher";
        publisher_->publish(message);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<TutPublisher>();

    // rclcpp::spin(publisher_node);
    std::thread([&publisher_node](){
        rclcpp::spin(publisher_node);
        std::cout << "Spin Ends" << std::endl;
    }).detach();

    std::cout << "Try publish message" << std::endl;
    publisher_node->publish();
    // rclcpp::shutdown();
    // std::cout << "ROS Ends" << std::endl;
    // sleep(10);
    return 0;
}