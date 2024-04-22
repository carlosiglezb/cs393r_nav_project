#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
// ROS Messages
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class TurtleBot4RechargeMonitorNode : public rclcpp::Node
{
public:
    TurtleBot4RechargeMonitorNode(const Eigen::Vector2f &docking_station_pos)
            : Node("turtlebot4_recharge_monitor_node")
    {
      docking_station_pos_ = docking_station_pos;

      // Model of battery discharge to distance
      min_battery_percentage_ = 0.15;     // minimum of 15% battery charge at end of trip
      battery_percentage_ = 1;            // full battery charge
      dist_per_charge_ = 20.;         // 20 meters per 1% charge
      dist_at_min_charge_ = dist_per_charge_ * min_battery_percentage_;     // ignore offset
      dist_offset_ = 0.0;             // offset for safety
      b_replan_to_docking_station_ = false;

      // Subscribe to the Path message to compute distance to the goal
      path_to_goal_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::path_distance_callback, this, std::placeholders::_1));

      // Subscribe to battery status
      battery_status_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::battery_status_callback, this, std::placeholders::_1));
    }

    void path_distance_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      if (b_replan_to_docking_station_) {
        return;
      }

      // Compute the distance to the goal
      float distance_to_goal = 0.0;
      for (int i = 0; i < msg->poses.size() - 1; i++) {
        distance_to_goal += sqrt(
                pow(msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x, 2) +
                pow(msg->poses[i+1].pose.position.y -  msg->poses[i].pose.position.y, 2));
      }

      // Compute distance goal to docking station
      float distance_goal_to_docking_station = sqrt(
              pow(docking_station_pos_[0] - msg->poses[msg->poses.size()-1].pose.position.x, 2) +
              pow(docking_station_pos_[1] - msg->poses[msg->poses.size()-1].pose.position.y, 2));

      float d_plan = distance_to_goal + distance_goal_to_docking_station;
      float d_available = dist_per_charge_ * battery_percentage_ - dist_offset_;
      // If total path distance is greater than the safe charge distance, then
      // the robot should return to the docking station
      if (d_available - d_plan > dist_at_min_charge_) {
        // finish plan
        b_replan_to_docking_station_ = false;
      } else {
        // re-plan and return to docking station
        b_replan_to_docking_station_ = true;
        // TODO call planner to docking station and set flag to dock
      }

    }

    void battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
      battery_percentage_ = msg->percentage;
    }

private:
    Eigen::Vector2f docking_station_pos_; // Docking station position (map frame)
    float min_battery_percentage_;        // Minimum desired battery percentage at docking station
    float battery_percentage_;            // Current battery percentage
    float dist_per_charge_;               // Slope of (available) distance (y) vs charge (x) curve
    float dist_at_min_charge_;            // Distance that can be traveled at min_battery_charge
    float dist_offset_;                   // Offset for safety
    bool b_replan_to_docking_station_;    // Flag to replan to docking stations
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  Eigen::Vector2f docking_station_pos(0.0, 0.0);
  rclcpp::spin(std::make_shared<TurtleBot4RechargeMonitorNode>(docking_station_pos));
  rclcpp::shutdown();
  return 0;
}