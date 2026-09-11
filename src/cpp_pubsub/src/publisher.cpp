#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;


class minimal_publisher : public rclcpp::Node {
public:
 minimal_publisher() : Node("publisher"), count(0){
    publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer = this->create_wall_timer(500ms, std::bind(&minimal_publisher::timer_callback, this));
 }

private:
    void timer_callback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " + std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
        publisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    size_t count;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<minimal_publisher>());
    rclcpp::shutdown();
    return 0;
}