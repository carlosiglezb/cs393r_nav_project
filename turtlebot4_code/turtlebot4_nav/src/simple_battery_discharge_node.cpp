#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

class SimpleBatteryDischargeNode : public rclcpp::Node
{
public:
    SimpleBatteryDischargeNode() : Node("simple_battery_discharge_node")
    {
        max_charge_ = 1.63;       // max charge: 1.63 Ah
        t_s_ = 0;

        // Create a timer to update battery state every second
        timer_ = this->create_wall_timer(
            1s, std::bind(&SimpleBatteryDischargeNode::newTimeCallback, this));

        // Create a publisher to publish the remapped cmd_vel message
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/ut/battery_state", 1);
    }

private:
    void newTimeCallback()
    {
        t_s_+=1;
        // update battery state by reducing charge by 0.00025 Ah every second (realistic discharge rate)
        // update battery state by reducing charge by 0.001 Ah every second (shortcut)
        auto updated_battery_state = sensor_msgs::msg::BatteryState();
        updated_battery_state.voltage = 12;     // mandatory field
        updated_battery_state.charge = max_charge_ - (0.001 * t_s_);
        battery_pub_->publish(updated_battery_state);
    }

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float max_charge_;    // max charge: 1.63 Ah
    float t_s_;              // time in seconds
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleBatteryDischargeNode>());
    rclcpp::shutdown();
    return 0;
}
