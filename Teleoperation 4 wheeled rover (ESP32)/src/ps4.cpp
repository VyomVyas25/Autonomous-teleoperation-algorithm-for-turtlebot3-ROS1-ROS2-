// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joy.hpp"

// class PS4ControllerSubscriberNode : public rclcpp::Node {
// public:
//     PS4ControllerSubscriberNode() : Node("ps4_controller_subscriber_node") {
//         // Create subscription to Joy messages
//         joy_subscription_ = this->create_subscription
//         <sensor_msgs::msg::Joy>(
//             "/joy",
//             10,
//             std::bind(&PS4ControllerSubscriberNode::joyCallback, this, std::placeholders::_1)
//         );
//     }

// private:
//     void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
//         // Check button states
//         for (size_t i = 0; i < msg->buttons.size(); ++i) {
//             if (msg->buttons[i] == 1) {
//                 RCLCPP_INFO(this->get_logger(), "Button %zu pressed", i);
//             }
//         }
//     }

//     rclcpp::Subscription
//     <sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared
//     <PS4ControllerSubscriberNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PS4BotControl : public rclcpp::Node
{
public:
    PS4BotControl()
    : Node("ps4_bot_control")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&PS4BotControl::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->axes[1];  // Left joystick vertical axis
        twist.angular.z = msg->axes[4]; // Right joystick horizontal axis
        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PS4BotControl>());
    rclcpp::shutdown();
    return 0;
}

