#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <bits/stdc++.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class TurtlBot3 : public rclcpp::Node
{
public:
    TurtlBot3()
        : Node("minmal_publsiher"), count_(0)
    {
        subscription_1 = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TurtlBot3::topic_callback, this, _1));
        subscription_2 = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TurtlBot3::obs_avoid, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&TurtlBot3::timer_callback, this));
        TurtlBot3::set_goal();
    }
    vector<float> scan;

private:
    float x_self, y_self, x_goal, y_goal, theta_self;
    long double x, dis_1 = 0, sum = 0, m, n, max, dist;
    void topic_callback(nav_msgs::msg::Odometry msg)
    {
        x_self = msg.pose.pose.position.x;
        y_self = msg.pose.pose.position.y;
        theta_self = quatToEuler(msg.pose.pose.orientation);
    }

    void obs_avoid(const sensor_msgs::msg::LaserScan &msg)
    {
        long double i = 0, k = 0;
        sum = 0;
        for (long double x : msg.ranges) // LiDAR data
        {
            if (x > 2)
            {
                x = 2;
            }
            dis_1 = 1 - x / 2;
            max = (1 - dis_1) * 2;
            scan.push_back(dis_1);
            if (i < 121 || i > 239)
            {
                if (i < 121)
                {
                    k = i;
                }
                else
                {
                    k = i - 359;
                }
                sum -= 3 * dis_1 * k; //calculation of angle of obstacle
            }
            // cout << sum << endl;
            i++;
        }
        scan.clear();
    }

    void set_goal()
    {
        cout << "Enter x coord:" << endl;
        cin >> x_goal;
        cout << "Enter y coord:" << endl;
        cin >> y_goal;
        dist = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2)); // dist of initial posn and final posn
    }

    double quatToEuler(const geometry_msgs::msg::Quaternion &quaternion)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);   // transformation 
        return yaw;
    }

    void timer_callback()
    {
        double dis, angle, vel = 0, angle_diff = 0, k_lin, obs_lin, obs_ang, k_ang, goal_ang;
        auto message = geometry_msgs::msg::Twist();
        dis = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2));
        if (dis > 0.01)
        {
            angle = atan2(y_goal - y_self, x_goal - x_self) - theta_self;
            if (angle > 3.14)
            {
                angle = angle - 6.28;
            }
            else if (angle < -3.14)
            {
                angle = angle + 6.28; // angle for avoiding infinity
            }
            angle_diff = 0.45 * angle; // angle differnce of the current posn and goal posn
            if (angle_diff > 0.4)
            {
                angle_diff = 0.4 * angle_diff / abs(angle_diff); // limiting goal angular velocity
            }

            vel = (dis / dist);
            if (vel > 0.5)
            {
                vel = 0.5;                                     //limiting goal linear velocity
            }

            obs_lin = (exp(this->dis_1) - 2);
            k_lin = 0.7 - exp(obs_lin);                        // calculator for linear velocity due to obstacle

            goal_ang = 4 / (1 + exp(-500 * (max - 2)));        // calculator for goal angular velocity
            obs_ang = abs(7.389 - exp(goal_ang)) * sum / 600;
            k_ang = 40 * obs_ang + 0.2 * goal_ang * angle_diff;

            message.linear.x = 4 * k_lin * vel;                // adjusting linear velocity
            message.angular.z = k_ang;
            if (abs(message.angular.z) > 0.5)
            {
                message.angular.z = 0.5 * abs(message.angular.z) / message.angular.z; // limiting the resultant angular velocity
            }
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "lin_vel : %lf", vel);
            // RCLCPP_INFO(this->get_logger(), "ang_vel : %lf", angle_diff);
        }

        else
        {
            set_goal();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_2;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlBot3>());
    rclcpp::shutdown();
    return 0;
}
