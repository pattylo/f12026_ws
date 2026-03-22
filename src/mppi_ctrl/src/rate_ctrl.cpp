#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class RateCtrlNode : public rclcpp::Node {
public:
    RateCtrlNode() : Node("servo_control_node") 
    {


        rc_pub_ = this->create_publisher<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in",
            10
        );


        
        // Timer for motor control updates
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RateCtrlNode::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "ctrl via feedback");
    }
    
    ~RateCtrlNode() {
    }


private:
    // std::unique_ptr<Motor> motor_;
    // std::vector<double> desired_manual_;
    // rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rate_sub_;
    mavros_msgs::msg::RCIn rate_pub_obj;
    geometry_msgs::msg::PoseStamped pose_msg;

    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub_;
    // rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg)
    {
        pose_msg = *msg;
    }
    
    void controlLoop() 
    {
        rate_pub_obj.channels.clear();
        rate_pub_obj.channels.emplace_back(1650);
        rate_pub_obj.channels.emplace_back(1100);
        rc_pub_->publish(rate_pub_obj);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RateCtrlNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    // node->shutdown();
    rclcpp::shutdown();
    
    return 0;
}
