#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <iostream>
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"


class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
    : Node("odom_publisher"),
      last_time_(this->now()),
      x_(0.0),
      y_(0.0),
      theta_(0.0)
    {
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OdomPublisher::cmdVelCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust the period as needed
            std::bind(&OdomPublisher::update, this));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_ = *msg;  // Store the received message
    }

    void update()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        Controller(last_cmd_, dt, current_time);
        last_time_ = current_time;
    }

    void Controller(const geometry_msgs::msg::Twist& msg, double dt, rclcpp::Time current_time)
    {
        std::cout <<msg.linear.x <<std::endl;

        double delta_x = msg.linear.x * dt * cos(theta_);
        double delta_y = msg.linear.x * dt * sin(theta_);
        double delta_theta = msg.angular.z * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Create an Odometry message and populate it with the computed values
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Set the position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Convert theta to a quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        // Set the velocity
        odom.twist.twist.linear.x = msg.linear.x;
        odom.twist.twist.linear.y = msg.linear.y;
        odom.twist.twist.angular.z = msg.angular.z;

        // Publish the odometry message
        odom_publisher_->publish(odom);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    double x_, y_, theta_;
    geometry_msgs::msg::Twist last_cmd_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
