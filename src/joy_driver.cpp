#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <std_srvs/srv/trigger.hpp>
using std::placeholders::_1;

class JoyDriver : public rclcpp::Node
{
  public:
    JoyDriver()
    : Node("a1_joy_driver")
    {
        twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyDriver::joyCallback, this, _1));

        change_mode_client_ = this->create_client<std_srvs::srv::Trigger>("/change_mode");
        
        // Wait for the service to be available
        while (!change_mode_client_->wait_for_service(std::chrono::seconds(1)))
        {
          if (!rclcpp::ok())
          {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
    }

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMessage)
    {
        auto twistMessage = geometry_msgs::msg::Twist();
        twistMessage.angular.z = joyMessage->axes[0];
        twistMessage.linear.x = joyMessage->axes[1];
        twistMessage.linear.y = joyMessage->axes[3];

        if (joyMessage->buttons[9] == 1) {
          // Prepare the request (empty request, nothing to add)
          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

          // Call the service
          auto future = change_mode_client_->async_send_request(request);

          // Wait for the response
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
              rclcpp::executor::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "Service call successful. Empty service triggered.");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
          }
        }

        twistPub_->publish(twistMessage);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_mode_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriver>());
  rclcpp::shutdown();
  return 0;
}