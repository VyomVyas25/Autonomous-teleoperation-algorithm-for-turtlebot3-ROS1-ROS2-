#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RoboticArmControl : public rclcpp::Node
{
public:
    RoboticArmControl()
    : Node("robotic_arm_control")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_1", 10);
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RoboticArmControl::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        // Button presses determine PWM velocity
        twist.linear.x = joy_msg->buttons[0] ? 1.0 : 0.0;  // Button 0 controls motor 1
        twist.linear.y = joy_msg->buttons[1] ? 1.0 : 0.0;  // Button 1 controls motor 2
        twist.linear.z = joy_msg->buttons[2] ? 1.0 : 0.0;  // Button 2 controls motor 3
        twist.angular.x = joy_msg->buttons[3] ? 1.0 : 0.0; // Button 3 controls motor 4
        twist.angular.y = joy_msg->buttons[4] ? 1.0 : 0.0; // Button 4 controls motor 5
        twist.angular.z =  joy_msg->axes[1];               // Joystick vertical axis for direction

        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboticArmControl>());
    rclcpp::shutdown();
    return 0;
}
