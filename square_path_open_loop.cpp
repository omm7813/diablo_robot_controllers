#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;
using std::placeholders::_1;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("diablo_square_path_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");
                
        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);
    
        
        move_forward_ = true;
        timer_start_time_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DiabloSquarePathNode::start, this));


    }

    void start() {

        //Make the robot stands up
        motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = true;
                cmd.value.up = 0.2;
                cmd.mode.stand_mode = true;
                motion_publisher_->publish(cmd);

        // If the robot is moving forward (as initially set), check the distance moved and change direction if necessary
        if (move_forward_)
        {
            if ((now() - timer_start_time_).seconds() <= 5)
            {
                // Publish forward motion command
                motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = false;
                cmd.value.forward = 0.2; // Assigning only value for moving forward
                cmd.value.left = 0.2;
                cmd.value.up = 0.2;
                cmd.value.roll = 0.0;
                cmd.value.pitch = 0.0;
                cmd.value.leg_split = 0.0;
                motion_publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Moving forward message is published!");
                RCLCPP_INFO(this->get_logger(), "Current time difference is: %.2f",(now() - timer_start_time_).seconds());
            }
            else
            {
                // Stop the robot after moving 1 meter
                move_forward_ = false;
                timer_start_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "The robot has stopped .. initializing the rotation");
            }
        }
        else
        {
            
            if ((now() - timer_start_time_).seconds() <= 7.85)
            {
                // Publish rotation command (90 degrees)
                motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = false;
                cmd.value.forward = 0.0;
                cmd.value.left = 0.2;
                cmd.value.up = 0.2;
                cmd.value.roll = 0.0;
                cmd.value.pitch = 0.0;
                cmd.value.leg_split = 0.0;
                cmd.mode.stand_mode =  true;
                motion_publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Rotation message is published!");
                RCLCPP_INFO(this->get_logger(), "Current time difference is: %.2f",(now() - timer_start_time_).seconds());
            }
          else {
            //Stop rotation
            motion_msgs::msg::MotionCtrl cmd;
            cmd.mode_mark = false;
            cmd.value.forward = 0.0;
            cmd.value.left = 0.0;
            cmd.value.up = 0.2;
            cmd.value.roll = 0.0;
            cmd.value.pitch = 0.0;
            cmd.value.leg_split = 0.0;
            cmd.mode.stand_mode =  true;
            motion_publisher_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Rotation stopped! .. initializing the forward movement again ..");

            timer_start_time_ = this->now();        
            move_forward_ = true;
            }
        }
     }
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr diablo_position_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool move_forward_;
    rclcpp::Time timer_start_time_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
