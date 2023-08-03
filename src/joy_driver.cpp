#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/joy.hpp"
using std::placeholders::_1;

class JoyDriver : public rclcpp::Node
{
  public:
    JoyDriver()
    : Node("minimal_subscriber")
    {
        twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyDriver::joyCallback, this, _1));
    }

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMessage) const
    {
        auto twistMessage = geometry_msgs::msg::Twist();
        twistMessage.angular.z = joyMessage->axes[0];
        twistMessage.linear.x = joyMessage->axes[1];
        twistMessage.linear.y = joyMessage->axes[3];

        twistPub_->publish(twistMessage);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriver>());
  rclcpp::shutdown();
  return 0;
}