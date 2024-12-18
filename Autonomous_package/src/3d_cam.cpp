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
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class CamNode : public rclcpp::Node
{
public:
    CamNode() : Node("CamNode") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10, std::bind(&CamNode::dataNode, this, _1));
    }

private:
float depth;

void dataNode(const sensor_msgs::msg::Image &msg){
    int i = 0;
    for(double x : msg.data){
        depth = x;
        cout << depth << " ";
        //cout << size(msg.data) << " " << endl;
        i++;
    }
    
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamNode>());
    rclcpp::shutdown();
    return 0;
}