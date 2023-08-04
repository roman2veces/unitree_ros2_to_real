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
        twist_subs_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&A1TwistDriver::driver, this, std::placeholders::_1));
        // change_mode_subs_ = this->create_subscription<std_msgs::msg::Int8>("mode", 10, std::bind(&A1TwistDriver::changeMode, this, std::placeholders::_1));
        change_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/change_mode",
            std::bind(&A1TwistDriver::changeMode, this, std::placeholders::_1, std::placeholders::_2)
        );
    
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

private:
    void driver(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd;
        ros_high_cmd.mode = 2;
        ros_high_cmd.forward_speed = msg->linear.x;
        ros_high_cmd.side_speed = msg->linear.y;
        ros_high_cmd.body_height = msg->linear.z;
        ros_high_cmd.rotate_speed = msg->angular.z;

        // TODO: map yaw, pitch and roll
        SendHighLCM = ToLcm(ros_high_cmd, SendHighLCM);
        roslcm.Send(SendHighLCM);
    }

    // Low level = 1 
    // High level = 2 
    // At this moment, there is not a way to change to sport mode in the SDK
    // void changeMode(const std_msgs::msg::Int8::SharedPtr msg)
    // {
    //     // ros_high_cmd.forward_speed = 0.0f;
    //     // ros_high_cmd.side_speed = 0.0f;
    //     // ros_high_cmd.rotate_speed = 0.0f;

    //     if (msg->data != 1 && msg->data != 2)
    //         return;
        
    //     ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd;
    //     ros_high_cmd.mode = msg->data;

    //     // TODO: map yaw, pitch and roll
    //     SendHighLCM = ToLcm(ros_high_cmd, SendHighLCM);
    //     roslcm.Send(SendHighLCM);
    // }

    void changeMode(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd;

        if (is_walking) {
            ros_high_cmd.mode = 1;
            is_walking = false;
        } else {
            ros_high_cmd.mode = 2;
            is_walking = true;
        }

        SendHighLCM = ToLcm(ros_high_cmd, SendHighLCM);
        roslcm.Send(SendHighLCM);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subs_;
    // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr change_mode_subs_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_mode_srv_;

    bool is_walking = false;
};

int
main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ROS2 Setup
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A1TwistDriver>();
    rclcpp::WallRate loop_rate(500);
    // TODO: understand why we need a SingleThreadedExecutor
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

        executor.spin_some();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}