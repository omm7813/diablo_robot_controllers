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
    DiabloSquarePathNode() : Node("p_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        diablo_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/diablo/pose", 10, std::bind(&DiabloSquarePathNode::poseCallback, this, std::placeholders::_1));

        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);

        // Initialize control parameters
        forward_speed_ = 0.0;       // Forward speed
        rotation_speed_ = 0.0;      // Rotation speed
        position_error = 0.0;
        rotation_error = 0.0;
        Kv = 0.4;       // Forward gain factor
        Kw = 0.2;      // Rotation gain factor
        rotation_tolerance_ = 0.1;  // Rotation tolerance in radians
        position_tolerance_ = 0.05; // Position tolerance in meters
        
        initial_x = 0.0;
        initial_y = 0.0;
        x_c = 0.0;
        y_c = 0.0;
        yaw_c = 0.0;

        firstMsg = true;
        move_forward_ = true;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DiabloSquarePathNode::start, this));
    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;

        //create a variable to store the current yaw reading
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = atan2(siny_cosp, cosy_cosp);

        //Extract position readings
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        x_c = x;
        y_c = y;
        yaw_c = yaw;

        if (firstMsg)
        {
            initial_x = x;
            initial_y = y;
            firstMsg = false;
        }

    }

    enum RobotState {
    STAND_UP,
    ROTATE_TO_TARGET_1,
    MOVE_TO_TARGET_1,
    ROTATE_TO_TARGET_2,
    MOVE_TO_TARGET_2,
    ROTATE_TO_TARGET_3,
    MOVE_TO_TARGET_3,
    ROTATE_TO_TARGET_4,
    MOVE_TO_TARGET_4,
    DONE
    };

    void start()
    {
    
        static RobotState current_state = STAND_UP;

        switch (current_state) 
        {
            case STAND_UP:
            {
                // Make the robot stand up
                motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = true;
                cmd.value.up = 0.2;
                cmd.mode.stand_mode = true;
                motion_publisher_->publish(cmd);
                current_state = ROTATE_TO_TARGET_1;
                break;
            }

            case ROTATE_TO_TARGET_1:
            {
                if (rotate_to(initial_x + 1, initial_y, initial_x, initial_y)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Rotated to target point 1! \n Initiating Forward Motion..");
                    current_state = MOVE_TO_TARGET_1;
                }
                break;
            }

            case MOVE_TO_TARGET_1:
            {
                if (move_to(initial_x + 1, initial_y, initial_x, initial_y)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Moved to target point 1! \n Initiating Rotation..");
                    current_state = ROTATE_TO_TARGET_2;
                }
                break;
            }

            case ROTATE_TO_TARGET_2:
            {
                if (rotate_to(initial_x + 1, initial_y+1, initial_x + 1, initial_y)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Rotated to target point 2! \n Initiating Forward Motion..");
                    current_state = MOVE_TO_TARGET_2;
                }
                break;
            }

            case MOVE_TO_TARGET_2:
            {
                if (move_to(initial_x + 1, initial_y + 1, initial_x + 1, initial_y)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Moved to target point 2! \n Initiating Rotation..");
                    current_state = ROTATE_TO_TARGET_3;
                }
                break;
            }

            case ROTATE_TO_TARGET_3:
            {
                if (rotate_to(initial_x, initial_y + 1, initial_x + 1, initial_y + 1)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Rotated to target point 3! \n Initiating Forward Motion..");
                    current_state = MOVE_TO_TARGET_3;
                }
                break;
            }

            case MOVE_TO_TARGET_3:
            {
                if (move_to(initial_x, initial_y + 1, initial_x + 1, initial_y + 1)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Moved to target point 3! \n Initiating Rotation..");
                    current_state = ROTATE_TO_TARGET_4;
                }
                break;
            }

            case ROTATE_TO_TARGET_4:
            {
                if (rotate_to(initial_x, initial_y, initial_x, initial_y + 1)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Rotated to target point 4! \n Initiating Forward Motion..");
                    current_state = MOVE_TO_TARGET_4;
                }
                break;
            }

            case MOVE_TO_TARGET_4:
            {
                if (move_to(initial_x, initial_y, initial_x, initial_y + 1)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Moved to target point 4! \n Initiating Rotation..");
                    current_state = DONE;
                }
                break;
            }
            case DONE:
            {
                // Make the robot stand up
                motion_msgs::msg::MotionCtrl cmd;
                cmd.mode_mark = true;
                cmd.value.up = 0.0;
                cmd.value.forward = 0.0;
                cmd.value.left = 0.0;
                motion_publisher_->publish(cmd);
                break;
            }
        }
        
    }


    bool move_to (double xt, double yt,  double xc, double yc) {
        position_error = ((cos(yaw_c)*(xt-x_c))+(sin(yaw_c)*(yt-y_c)));
        rotation_error = ((cos(yaw_c)*(yt-y_c))-(sin(yaw_c)*(xt-x_c)));
        forward_speed_ =   Kv * position_error;
        rotation_speed_ =  Kw * rotation_error;

        RCLCPP_INFO(this->get_logger(), "Moving Forward..");
        RCLCPP_INFO(this->get_logger(), "initial x: %.2f, initial y: %.2f",xc, yc);
        RCLCPP_INFO(this->get_logger(), "target x: %.2f, target y: %.2f",xt, yt);
        RCLCPP_INFO(this->get_logger(), "position_error is: %.2f",position_error);
        RCLCPP_INFO(this->get_logger(), "rotation_error is: %.2f",rotation_error);
        RCLCPP_INFO(this->get_logger(), "forward_speed_ is: %.2f",forward_speed_);
        RCLCPP_INFO(this->get_logger(), "rotation_speed_ is: %.2f \n",rotation_speed_);
       

        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = forward_speed_;
        cmd.value.left = rotation_speed_;
        cmd.value.up = 0.2;
        motion_publisher_->publish(cmd);

        if (abs(position_error) <= position_tolerance_ && abs(rotation_error) <= rotation_tolerance_) {
            motion_msgs::msg::MotionCtrl cmd;
            cmd.mode_mark = false;
            cmd.value.forward = 0.0;
            cmd.value.left = 0.0;
            cmd.value.up = 0.2;
            motion_publisher_->publish(cmd);

            return true;
        } else return false;
    }

    //t is for target
    //c is for current
    bool rotate_to (double xt, double yt, double xc, double yc) {
        position_error = ((cos(yaw_c)*(xc-x_c))+(sin(yaw_c)*(yc-y_c)));
        rotation_error = ((cos(yaw_c)*(yt-y_c))-(sin(yaw_c)*(xt-x_c)));
        forward_speed_ = Kv * position_error;
        rotation_speed_ = Kw * rotation_error;

        RCLCPP_INFO(this->get_logger(), "Rotating..");
        RCLCPP_INFO(this->get_logger(), "initial x: %.2f, initial y: %.2f",xc, yc);
        RCLCPP_INFO(this->get_logger(), "target x: %.2f, target y: %.2f",xt, yt);
        RCLCPP_INFO(this->get_logger(), "position_error is: %.2f",position_error);
        RCLCPP_INFO(this->get_logger(), "rotation_error is: %.2f",rotation_error);
        RCLCPP_INFO(this->get_logger(), "forward_speed_ is: %.2f",forward_speed_);
        RCLCPP_INFO(this->get_logger(), "rotation_speed_ is: %.2f \n",rotation_speed_);
        

        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = forward_speed_;
        cmd.value.left = rotation_speed_;
        cmd.value.up = 0.2;
        motion_publisher_->publish(cmd);

        if (abs(position_error) <= position_tolerance_ && abs(rotation_error) <= rotation_tolerance_) {
            motion_msgs::msg::MotionCtrl cmd;
            cmd.mode_mark = false;
            cmd.value.forward = 0.0;
            cmd.value.left = 0.0;
            cmd.value.up = 0.2;
            motion_publisher_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Rotated.");
            return true;
        } else return false;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool move_forward_, firstMsg;
    double initial_x;
    double initial_y;
    double forward_speed_;
    double rotation_speed_;
    double rotation_tolerance_;
    double position_tolerance_;
    double Kv;
    double Kw;
    double position_error;
    double rotation_error;
    double x_c, y_c, yaw_c;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
