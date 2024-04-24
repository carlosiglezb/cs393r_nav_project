#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Dense"
// ROS Messages
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
// Actions
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "irobot_create_msgs/action/dock.hpp"

class TurtleBot4RechargeMonitorNode : public rclcpp::Node
{
public:
    TurtleBot4RechargeMonitorNode(const Eigen::Vector2f &docking_station_pos)
            : Node("turtlebot4_recharge_monitor_node")
    {
      docking_station_pos_ = docking_station_pos;
      float max_charge = 1.63;             // maximum battery charge [Ah]

      // Model of battery discharge to distance
      float min_battery_percentage_ = 0.20;     // minimum of 20% battery charge at end of trip
      min_safe_charge = min_battery_percentage_ * max_charge;     // min allowed charge at dock
      battery_charge_ = 1.63;             // assume full battery charge (get updated at next tick)
      dist_per_charge_ = 570.;            // 570 meters per Ah, obtained from data collection
      b_dock_visible_ = false;
      b_replan_to_docking_station_ = false;
      b_prepare_to_dock_ = false;
      b_is_docked_ = false;

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

      // Subscribe to dock status to check if docking station is visible
      dock_status_subscriber_ = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
            "/dock_status",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::dock_status_callback, this, std::placeholders::_1));

      // Client to cancel current plan
      nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

      // Client to send dock command
      dock_client_ = rclcpp_action::create_client<irobot_create_msgs::action::Dock>(this, "/dock");

      // Debug
      replan_publisher_ = this->create_publisher<std_msgs::msg::Bool>("debug/replan_called", 5);
    }

    void path_distance_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      // TODO check overall logic of flags
      // reset flags and exit if docked
      if (b_is_docked_) {
        b_prepare_to_dock_ = false;
        b_replan_to_docking_station_ = false;
        return;
      }

      // dock if requested and we are near docking station
      if (b_prepare_to_dock_) {
        if (b_dock_visible_) {
          send_dock_command();
        }
      }

      // if we are on our way to the docking station, no need to estimate distance-to-discharge
      if (b_replan_to_docking_station_) {
        return;
      }

      // Compute the distance to the goal
      float distance_to_goal = 0.0;
      for (unsigned int i = 0; i < msg->poses.size() - 1; i++) {
        distance_to_goal += sqrt(
                pow(msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x, 2) +
                pow(msg->poses[i+1].pose.position.y -  msg->poses[i].pose.position.y, 2));
      }

      // Compute distance goal to docking station
      float distance_goal_to_docking_station = sqrt(
              pow(docking_station_pos_[0] - msg->poses[msg->poses.size()-1].pose.position.x, 2) +
              pow(docking_station_pos_[1] - msg->poses[msg->poses.size()-1].pose.position.y, 2));

      float d_plan = distance_to_goal + distance_goal_to_docking_station;
      float d_available = 4.;     // [FOR TESTING PURPOSES ONLY]
//      float d_available = dist_per_charge_ * (battery_charge_ - min_safe_charge);
      // If total path distance is greater than the safe charge distance, then
      // the robot should return to the docking station
      if (d_plan >= d_available) {
        // re-plan and return to docking station
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.pose.position.x = docking_station_pos_[0];
        goal.pose.pose.position.y = docking_station_pos_[1];
        replan_to_(goal);
        b_replan_to_docking_station_ = true;
        b_prepare_to_dock_ = true;
        return;
      }
      auto replan_msg = std_msgs::msg::Bool();
      replan_msg.data = false;
      replan_publisher_->publish(replan_msg);
    }

    void battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
//      battery_percentage_ = msg->percentage;
      battery_charge_ = msg->charge;
    }

    void dock_status_callback(const irobot_create_msgs::msg::DockStatus::SharedPtr msg)
    {
      b_dock_visible_ = msg->dock_visible;
      b_is_docked_ = msg->is_docked;
    }

    void replan_to_(const nav2_msgs::action::NavigateToPose::Goal &goal)
    {
      // cancel current plan
      nav_to_pose_client_->async_cancel_all_goals();
      std::cout << "Canceling current plan" << std::endl;

      // set new goal at docking station
      nav_to_pose_client_->async_send_goal(goal);
      std::cout << "Replanning to docking station" << std::endl;
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      replan_publisher_->publish(msg);
    }

    void send_dock_command()
    {
      auto msg = irobot_create_msgs::action::Dock::Goal();
      dock_client_->async_send_goal(msg);
    }

private:
    // Actions
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<irobot_create_msgs::action::Dock>::SharedPtr dock_client_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_to_goal_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_subscriber_;
    rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_status_subscriber_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_publisher_;

    // Properties
    Eigen::Vector2f docking_station_pos_; // Docking station position (map frame)
    float min_safe_charge;            // Minimum desired battery charge at docking station
    float battery_charge_;                // Current battery charge [Ah]
//    float battery_percentage_;            // Current battery percentage
    float dist_per_charge_;               // Slope of (available) distance (y) vs charge (x) curve
    float dist_offset_;                   // Offset for safety
    bool b_dock_visible_;                   // Flag to check if docking station is visible
    bool b_replan_to_docking_station_;    // Flag to replan to docking stations
    bool b_prepare_to_dock_;              // Flag to prepare to dock
    bool b_is_docked_;                    // Flag to check if robot is docked
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  Eigen::Vector2f docking_station_pos(0.0, -0.1);
  rclcpp::spin(std::make_shared<TurtleBot4RechargeMonitorNode>(docking_station_pos));
  rclcpp::shutdown();
  return 0;
}