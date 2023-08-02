#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.h"
#include "ros2_unitree_legged_msgs/msg/high_state.h"
#include "convert.h"

// TODO: find an other way to do this without global variables
// TODO: convert ros2 node to a class 
UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
UNITREE_LEGGED_SDK::HighCmd SendHighLCM = {0};

void driver(const geometry_msgs::msg::Twist::SharedPtr msg) {
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

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(rclcpp::ok){
        data->Recv();
        usleep(2000);
    }
}

// TCmd = Type command
// TState = Type state
template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ROS2 Setup
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_ros2_walk_example");
    rclcpp::WallRate loop_rate(500);
    // TODO: understand why we need a SingleThreadedExecutor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto twist_subs = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, driver);
    ros2_unitree_legged_msgs::msg::HighCmd SendHighROS;
    ros2_unitree_legged_msgs::msg::HighState RecvHighROS;

    // LCM Setup
    TState RecvHighLCM = {0};
    roslcm.SubscribeState();
    
    // Threads setup
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (rclcpp::ok()) {
        roslcm.Get(RecvHighLCM);
        // TODO: do something with the high state reception 
        RecvHighROS = ToRos(RecvHighLCM);

        executor.spin_some();
        loop_rate.sleep(); 
    }

    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[]) {
    std::string robot_name;
    UNITREE_LEGGED_SDK::LeggedType rname = UNITREE_LEGGED_SDK::LeggedType::A1;

    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}