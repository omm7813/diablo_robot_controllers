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

#define PI 3.141592653589793238462643383279502884197

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("diablo_square_path_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");
                
        diablo_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/diablo/pose", 10, std::bind(&DiabloSquarePathNode::positionCallback, this, std::placeholders::_1));

        diablo_pose_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/diablo/sensor/Imu", 10, std::bind(&DiabloSquarePathNode::poseCallback, this, std::placeholders::_1));
            
        //RCLCPP_INFO(this->get_logger(), "Subscribed to diablo/pose");

        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);
    
        
        //RCLCPP_INFO(this->get_logger(), "Publisher to diablo/MotionCmd created");
        move_forward_ = true;
        local_x = 0.0;
        yaw_deg = 0.0;
        initial_position = 0.0;
        initial_yaw = 0.0;
        firstMsg = true;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DiabloSquarePathNode::start, this));

    }
private:
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double x = msg->pose.position.x;
        if(firstMsg){
            initial_position = x;
            firstMsg = false;
        }
        double y = msg->pose.position.y;
        local_x = x * cos(yaw_deg*M_PI/180) + y * sin(yaw_deg*M_PI/180) ;
        double z = msg->pose.position.z;
        //RCLCPP_INFO(this->get_logger(), "Received Position readings");
 
     }

     void poseCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        //RCLCPP_INFO(this->get_logger(), "Received orientation readings");
 
        double pitch = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = atan2(siny_cosp, cosy_cosp);
        //Convert the yaw readings to degrees
        yaw_deg = yaw * (180/PI);
        double roll = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
     }

     void start() {

        //Make the robot stands up
        motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = true;
                cmd.value.forward = 0.0; // Assigning only value for moving forward
                cmd.value.left = 0.0;
                cmd.value.up = 0.2;
                cmd.value.roll = 0.0;
                cmd.value.pitch = 0.0;
                cmd.value.leg_split = 0.0;
                cmd.mode.stand_mode = true;
                motion_publisher_->publish(cmd);

        // If the robot is moving forward (as initially set), check the distance moved and change direction if necessary
        if (move_forward_)
        {
            double distance_to_move = 0.5; 
            double distance_moved = local_x - initial_position;
            if (abs(distance_moved) <= abs(distance_to_move))
            {
                // Publish forward motion command
                motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = false;
                cmd.value.forward = 0.2; // Assigning only value for moving forward
                cmd.value.left = 0.0;
                cmd.value.up = 0.2;
                cmd.value.roll = 0.0;
                cmd.value.pitch = 0.0;
                cmd.value.leg_split = 0.0;
                motion_publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Moving forward message is published!");
                RCLCPP_INFO(this->get_logger(), "Current position is: %.2f",local_x);
                RCLCPP_INFO(this->get_logger(), "Initial position is: %.2f",initial_position);
            }
            else
            {
                // Stop the robot after moving 1 meter
                move_forward_ = false;
                initial_yaw = yaw_deg;
                RCLCPP_INFO(this->get_logger(), "The robot has stopped .. initializing the rotation");
            }
        }
        else
        {
            double degree_to_rotate = 90.0;
            //double initial_yaw = yaw_deg;
            double rotated_yaw = yaw_deg - initial_yaw;
            rotated_yaw = (180/PI)* atan2(sin(rotated_yaw*(PI/180)),cos(rotated_yaw*(PI/180)));
            RCLCPP_INFO(this->get_logger(), "Yaw Moved is: %.2f",rotated_yaw);
            RCLCPP_INFO(this->get_logger(), "Initial Yaw is: %.2f",initial_yaw);
            RCLCPP_INFO(this->get_logger(), "Current Yaw is: %.2f",yaw_deg);
            if (rotated_yaw <= degree_to_rotate) {
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

            initial_position = local_x;         
            move_forward_ = true;
            }
        }
     }
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr diablo_position_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool move_forward_, firstMsg;
    double initial_position;
    double initial_yaw;
    double local_x;
    double yaw_deg;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


