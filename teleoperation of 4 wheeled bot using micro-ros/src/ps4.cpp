#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class PS4ControllerSubscriberNode : public rclcpp::Node {
public:
    PS4ControllerSubscriberNode() : Node("ps4_controller_subscriber_node") {
        // Create subscription to Joy messages
        joy_subscription_ = this->create_subscription
        <sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&PS4ControllerSubscriberNode::joyCallback, this, std::placeholders::_1)
        );
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Check button states
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            if (msg->buttons[i] == 1) {
                RCLCPP_INFO(this->get_logger(), "Button %zu pressed", i);
            }
        }
    }

    rclcpp::Subscription
    <sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared
    <PS4ControllerSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
