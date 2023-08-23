#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_unitree_legged_msgs/msg/high_cmd.h"
#include "ros2_unitree_legged_msgs/msg/high_state.h"
#include "convert.h"

// TODO: find a way to add this variables to the class
UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
UNITREE_LEGGED_SDK::HighCmd SendHighLCM = {0};

class A1TwistDriver : public rclcpp::Node
{
public:
    A1TwistDriver() : Node("a1_twist_driver")
    {
        this->declare_parameter("start_walking", false);
        is_walking = this->get_parameter("start_walking").as_bool();

        high_state_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("state", 10);
        twist_subs_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&A1TwistDriver::driver, this, std::placeholders::_1));
        change_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/change_mode",
            std::bind(&A1TwistDriver::changeMode, this, std::placeholders::_1, std::placeholders::_2));
    }

    // template <typename TLCM>
    static void *updateLoop(void *param)
    {
        UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
        while (rclcpp::ok)
        {
            data->Recv();
            usleep(2000);
        }
    }

    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subs_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_mode_srv_;

private:
    void driver(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd;

        if (is_walking)
        {
            ros_high_cmd.mode = 2;
            ros_high_cmd.forward_speed = msg->linear.x;
            ros_high_cmd.side_speed = msg->linear.y;
            ros_high_cmd.rotate_speed = msg->angular.z;
            ros_high_cmd.pitch = msg->angular.y;
        }
        else
        {
            ros_high_cmd.mode = 1;
            ros_high_cmd.yaw = msg->angular.z;
            ros_high_cmd.roll = msg->linear.y;
            ros_high_cmd.pitch = msg->angular.y;
            ros_high_cmd.body_height = msg->linear.x;
        }

        // TODO: map yaw, pitch and roll
        SendHighLCM = ToLcm(ros_high_cmd, SendHighLCM);
        roslcm.Send(SendHighLCM);
    }

    // At this moment, there is not a way to change to sport mode in the SDK.
    // So this function only change the mode from walking to standing up without walking.
    void changeMode(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd;

        if (is_walking)
        {
            ros_high_cmd.mode = 1;
            is_walking = false;
        }
        else
        {
            ros_high_cmd.mode = 2;
            is_walking = true;
        }

        SendHighLCM = ToLcm(ros_high_cmd, SendHighLCM);
        roslcm.Send(SendHighLCM);
    }

    bool is_walking = false;
};

int main(int argc, char *argv[])
{
    // ROS2 Setup
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A1TwistDriver>();
    rclcpp::WallRate loop_rate(500);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ros2_unitree_legged_msgs::msg::HighCmd SendHighROS;
    ros2_unitree_legged_msgs::msg::HighState RecvHighROS;

    // LCM Setup
    UNITREE_LEGGED_SDK::HighState RecvHighLCM = {0};
    roslcm.SubscribeState();

    // Threads setup
    pthread_t tid;
    pthread_create(&tid, NULL, node->updateLoop, &roslcm);

    while (rclcpp::ok())
    {
        roslcm.Get(RecvHighLCM);
        // TODO: do something with the high state reception
        RecvHighROS = ToRos(RecvHighLCM);
        node->high_state_pub_->publish(RecvHighROS);

        RCLCPP_INFO(node->get_logger(), "forward_speed: '%f'",  RecvHighROS.forward_speed);
        // RCLCPP_INFO(node->get_logger(), "side_speed: '%f'",  RecvHighROS.side_speed);
        // RCLCPP_INFO(node->get_logger(), "rotate_speed: '%f'",  RecvHighROS.rotate_speed);
        // RCLCPP_INFO(node->get_logger(), "body_height: '%f'",  RecvHighROS.body_height);
        // RCLCPP_INFO(node->get_logger(), "updown_speed: '%f'",  RecvHighROS.updown_speed);
        // RCLCPP_INFO(node->get_logger(), "forward_position: '%f'",  RecvHighROS.forward_position);
        // RCLCPP_INFO(node->get_logger(), "side_position: '%f'",  RecvHighROS.side_position);   

        executor.spin_some();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}