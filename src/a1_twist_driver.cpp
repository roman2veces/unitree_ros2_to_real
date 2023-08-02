#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
    }

    // template <typename TLCM>
    static void *update_loop(void *param)
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
        ros2_unitree_legged_msgs::msg::HighCmd ros_high_cmd_;
        ros_high_cmd_.mode = 2;
        ros_high_cmd_.forward_speed = msg->linear.x;
        ros_high_cmd_.side_speed = msg->linear.y;
        ros_high_cmd_.body_height = msg->linear.z;
        ros_high_cmd_.rotate_speed = msg->angular.z;

        // TODO: map yaw, pitch and roll
        SendHighLCM = ToLcm(ros_high_cmd_, SendHighLCM);
        roslcm.Send(SendHighLCM);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subs_;
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
    pthread_create(&tid, NULL, node->update_loop, &roslcm);

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