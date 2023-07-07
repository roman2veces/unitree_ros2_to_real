#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.h"
#include "ros2_unitree_legged_msgs/msg/high_state.h"

#include "convert.h"

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(rclcpp::ok){
        data->Recv();
        usleep(2000);
    }
}


template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();


    // ros::NodeHandle n;
    // ros::Rate loop_rate(500);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_ros2_walk_example");
    rclcpp::WallRate loop_rate(500);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    // unitree_legged_msgs::HighCmd SendHighROS;
    ros2_unitree_legged_msgs::msg::HighCmd SendHighROS;
    ros2_unitree_legged_msgs::msg::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (rclcpp::ok()){
        motiontime = motiontime + 2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("%f\n",  RecvHighROS.forwardSpeed);

        SendHighROS.forward_speed = 0.0f;
        SendHighROS.side_speed = 0.0f;
        SendHighROS.rotate_speed = 0.0f;
        SendHighROS.body_height = 0.0f;

        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        if(motiontime>1000 && motiontime<1500){
            SendHighROS.mode = 1;
            // SendHighROS.roll = 0.3f;
        }

        if(motiontime>1500 && motiontime<2000){
            SendHighROS.mode = 1;
            SendHighROS.pitch = 0.3f;
        }

        if(motiontime>2000 && motiontime<2500){
            SendHighROS.mode = 1;
            SendHighROS.yaw = 0.2f;
        }

        if(motiontime>2500 && motiontime<3000){
            SendHighROS.mode = 1;
            SendHighROS.body_height = -0.3f;
        }

        if(motiontime>3000 && motiontime<3500){
            SendHighROS.mode = 1;
            SendHighROS.body_height = 0.3f;
        }

        if(motiontime>3500 && motiontime<4000){
            SendHighROS.mode = 1;
            SendHighROS.body_height = 0.0f;
        }

        if(motiontime>4000 && motiontime<5000){
            SendHighROS.mode = 2;
        }

        if(motiontime>5000 && motiontime<8500){
            SendHighROS.mode = 2;
            SendHighROS.forward_speed = 0.1f; // -1  ~ +1
        }

        if(motiontime>8500 && motiontime<12000){
            SendHighROS.mode = 2;
            SendHighROS.forward_speed = -0.2f; // -1  ~ +1
        }

        if(motiontime>12000 && motiontime<16000){
            SendHighROS.mode = 2;
            SendHighROS.rotate_speed = 0.1f;   // turn
        }

        if(motiontime>16000 && motiontime<20000){
            SendHighROS.mode = 2;
            SendHighROS.rotate_speed = -0.1f;   // turn
        }

        if(motiontime>20000 && motiontime<21000){
            SendHighROS.mode = 1;
        }

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        // ros::spinOnce();
        executor.spin_some();
        loop_rate.sleep(); 
    }

    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[]) {
    // #ifdef SDK3_1
    //     aliengo::Control control(aliengo::HIGHLEVEL);
    //     aliengo::LCM roslcm;
    //     mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    // #endif

    // #ifdef SDK3_2
    std::string robot_name;
    UNITREE_LEGGED_SDK::LeggedType rname = UNITREE_LEGGED_SDK::LeggedType::A1;

    // UNITREE_LEGGED_SDK::InitEnvironment();
    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    // #endif
    
}