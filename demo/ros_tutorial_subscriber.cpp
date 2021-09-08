//
// Created by yongxi on 2021/9/6.
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

class TutSubscriber : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    static void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << "Receive message: " << msg->data.c_str() << std::endl;
    }
public:
    TutSubscriber(): Node("tut_subscriber"){
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "test_topic",
                10,
                // std::bind(&TutSubscriber::topic_callback, this, std::placeholders::_1)
                TutSubscriber::topic_callback
                );
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TutSubscriber>());
    rclcpp::shutdown();
    return 0;
}