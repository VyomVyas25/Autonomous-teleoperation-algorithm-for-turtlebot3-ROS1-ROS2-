#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <iostream>

using namespace std;


class PS4ControllerSubscriberNode : public rclcpp::Node {
public:
    PS4ControllerSubscriberNode() : Node("ps4_controller_subscriber_node") {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&PS4ControllerSubscriberNode::joyCallback, this, std::placeholders::_1));
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Left Stick X: %f", msg->axes[0]);
        RCLCPP_INFO(this->get_logger(), "Left Stick Y: %f", msg->axes[1]);
        RCLCPP_INFO(this->get_logger(), "Right Stick X: %f", msg->axes[2]);
        RCLCPP_INFO(this->get_logger(), "Right Stick Y: %f", msg->axes[3]);

        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Button %zu: %d", i, msg->buttons[i]);
        }
       
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PS4ControllerSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
