#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class IrcNode : public rclcpp::Node {
public:
    IrcNode() : Node("ircNode"), state_(State::MOVING_FORWARD), count_(0) {
        subscription_0 = this->create_subscription<geometry_msgs::msg::Twist>("/arrow_info", 10, std::bind(&IrcNode::arr_detNode, this, _1));
        subscription_1 = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&IrcNode::arrowNode, this, _1));
        subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&IrcNode::thetaNode, this, _1) );
        publisher_1 = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(50ms, std::bind(&IrcNode::timer_callback, this));
        start_time_ = this->get_clock()->now();
    }

private:
    enum class State { MOVING_FORWARD, STOPPING, ROTATING };
    float direction, deviation, linear_velocity_ = 0.5, angular_velocity_ = 0.5, angular_velocity_2, sum, goal_ang, theta, theta_new;
    std::chrono::duration<float> stop_duration_ = 5s;
    float rotation_angle_ = M_PI / 2;
    rclcpp::Time start_time_;
    State state_;

    void arr_detNode(const geometry_msgs::msg::Twist &message) {
        direction = message.linear.z;
        deviation = message.linear.y;
    }

    void arrowNode(const sensor_msgs::msg::LaserScan &msg) {
        sum = 0;
        for (size_t i = 0; i < msg.ranges.size(); ++i) {
            float x = (msg.ranges[i] > 3 || isinf(msg.ranges[i])) ? 3 : msg.ranges[i];
            if (i < 6 || i > msg.ranges.size() - 6) {
                float obj_dis = 1 - (x / 3);
                float k_lin = (0.45 - (exp(exp(obj_dis) - 3)));
                sum -= (i < 6 ? (1 - obj_dis) * i : -(1 - obj_dis) * (msg.ranges.size() - i));
                linear_velocity_ = 4.5 * k_lin * 0.5;
            }
        }
    }

    void thetaNode(const nav_msgs::msg::Odometry &msg) {
        theta = quatToEuler(msg.pose.pose.orientation);
    }

    double quatToEuler(const geometry_msgs::msg::Quaternion &quaternion) {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void timer_callback() {
        auto vel = geometry_msgs::msg::Twist();
        auto now = this->get_clock()->now();
        float elapsed_seconds = (now - start_time_).seconds();

        bool is_moving = state_ == State::MOVING_FORWARD;
        bool is_stopping = state_ == State::STOPPING;
        bool is_rotating = state_ == State::ROTATING;

        sum = sum / 10;
        
        vel.linear.x = (is_moving && linear_velocity_ >= 0.05) ? ((linear_velocity_ > 0.4) ? 0.4 : linear_velocity_) : 0.0;

        if(direction==1 || direction==-1){
            angular_velocity_2 = (deviation/150 + (abs(sum) > 0.3 ? 0.3 * (sum / abs(sum)) : sum));
            
            if (is_rotating) {
                vel.angular.z = direction*angular_velocity_;
            } 

            else if(is_stopping){
                vel.angular.z = 0;
            }
            else {
                float orientation_error = theta_new - theta;
                orientation_error = (orientation_error)>3.14?orientation_error-(6.28*(orientation_error/orientation_error)):orientation_error;
                vel.angular.z = angular_velocity_2 + 5*((abs(orientation_error) > 0.025) ? 0.5 * orientation_error : 0.0);
                vel.angular.z = abs(vel.angular.z)>0.5?0.5*abs(vel.angular.z)/vel.angular.z : vel.angular.z;
            }
        }
        else {
            angular_velocity_2 = 0;
            }

        if (angular_velocity_2 < 0.03) {
            angular_velocity_2 = 0;
        }
        if (is_moving && linear_velocity_ < 0.05) {
            state_ = State::STOPPING;
            start_time_ = now;
        } else if (is_stopping && elapsed_seconds >= stop_duration_.count()) {
            state_ = State::ROTATING;
            start_time_ = now;
        } else if (is_rotating && elapsed_seconds >= (rotation_angle_ / (angular_velocity_*0.95))) {
            state_ = State::MOVING_FORWARD;
            start_time_ = now;
            theta_new = theta; //updated
        } 

        publisher_1->publish(vel);
        cout << vel.linear.x << " " << vel.angular.z << endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IrcNode>());
    rclcpp::shutdown();
    return 0;
}
