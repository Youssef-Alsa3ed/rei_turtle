#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class minimal_subscriber : public rclcpp::Node {
public:
    minimal_subscriber() : Node("subscriber"){
        subscription = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&minimal_subscriber::topic_callback, this, std::placeholders::_1));
    }
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<minimal_subscriber>());
    rclcpp::shutdown();
    return 0;
}