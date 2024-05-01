#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class CmdVelRemapNode : public rclcpp::Node
{
public:
    CmdVelRemapNode() : Node("cmd_vel_remap_node")
    {
        // Create a publisher to publish the remapped cmd_vel message
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ut/cmd_vel", 2);

        // Create a subscriber to subscribe to the cmd_vel message
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SensorDataQoS(),
            std::bind(&CmdVelRemapNode::cmdVelCallback, this, std::placeholders::_1));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Remap the linear x velocity
        double x = msg->linear.x;
        double y = msg->linear.y;
        double z = msg->angular.z;

        // Publish the remapped cmd_vel message
        auto remapped_msg = geometry_msgs::msg::Twist();
        remapped_msg.linear.x = x;
        remapped_msg.linear.y = y;
        remapped_msg.angular.z = z;
        pub_->publish(remapped_msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelRemapNode>());
    rclcpp::shutdown();
    return 0;
}
