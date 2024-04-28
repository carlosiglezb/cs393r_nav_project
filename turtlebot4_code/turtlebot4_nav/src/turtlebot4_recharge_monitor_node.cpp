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
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "amrl_msgs/msg/turtlebot_dock_status.hpp"
//#include "irobot_create_msgs/msg/dock_status.hpp"
// Actions
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "amrl_msgs/action/turtlebot_dock.hpp"
//#include "irobot_create_msgs/action/dock.hpp"

class TurtleBot4RechargeMonitorNode : public rclcpp::Node
{
public:
    TurtleBot4RechargeMonitorNode(const Eigen::Vector2f &docking_station_pos)
            : Node("turtlebot4_recharge_monitor_node")
    {
      docking_station_pos_ = docking_station_pos;
//      docking_station_pos_map_ = Eigen::Vector2f(-5.5, 23.5);
      docking_station_pos_map_ = Eigen::Vector2f(-0.5, 0.0);
      robot_pos_ = Eigen::Vector2f(0.0, 0.0); // assume robot starts at origin
      float max_charge = 1.63;             // maximum battery charge [Ah]

      // Model of battery discharge to distance
      float min_battery_percentage_ = 0.60;     // minimum of 20% battery charge at end of trip
      min_safe_charge = min_battery_percentage_ * max_charge;     // min allowed charge at dock
      battery_charge_ = 1.63;             // assume full battery charge (get updated at next tick)
      dist_per_charge_ = 570.;            // 570 meters per Ah, obtained from data collection
      b_dock_visible_ = false;
      b_replan_to_docking_station_ = false;
      b_prepare_to_dock_ = false;
      b_is_docked_ = false;
      b_replan_to_docking_station_sent_ = false;

      // Subscribe to the Path message to compute distance to the goal
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::odom_callback, this, std::placeholders::_1));

      // Subscribe to the Path message to compute distance to the goal
      path_to_goal_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::path_distance_callback, this, std::placeholders::_1));

      // Subscribe to battery status
      battery_status_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/ut/battery_state",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::battery_status_callback, this, std::placeholders::_1));

      // Subscribe to dock status to check if docking station is visible
//      dock_status_subscriber_ = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
      dock_status_subscriber_ = this->create_subscription<amrl_msgs::msg::TurtlebotDockStatus>(
            "/ut/dock_status",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::dock_status_callback, this, std::placeholders::_1));

      // Remap CMD velocity to /ut/cmd_vel
      cmd_remap_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4RechargeMonitorNode::cmd_remap_callback, this, std::placeholders::_1));

      // Client to cancel current plan
      nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
//      nav_to_position_client_ = rclcpp_action::create_client<irobot_create_msgs::action::NavigateToPosition>(this, "/ut/navigate_to_position");

      // Client to send dock command
//      dock_client_ = rclcpp_action::create_client<irobot_create_msgs::action::Dock>(this, "/dock");
      dock_client_ = rclcpp_action::create_client<amrl_msgs::action::TurtlebotDock>(this, "/ut/turtlebot_dock");

      // Debug
      replan_publisher_ = this->create_publisher<std_msgs::msg::Bool>("debug/replan_called", 1);
      near_docking_publisher_ = this->create_publisher<std_msgs::msg::Bool>("debug/near_docking", 1);
      d_available_publisher_ = this->create_publisher<std_msgs::msg::Float32>("debug/d_available", 1);
      d_plan_publisher_ = this->create_publisher<std_msgs::msg::Float32>("debug/d_plan", 1);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      robot_pos_ = Eigen::Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    void path_distance_callback(nav_msgs::msg::Path::SharedPtr msg)
    {
      // TODO check overall logic of flags
      // reset flags and exit if docked
      if (b_is_docked_) {
        b_prepare_to_dock_ = false;
        b_replan_to_docking_station_ = false;
        b_replan_to_docking_station_sent_ = false;
        std::cout << "Docked. Resetting flags." << std::endl;
        return;
      }

      // if we are on our way to the docking station, no need to estimate distance-to-discharge
//      bool b_near_docking_station = (robot_pos_ - docking_station_pos_).norm() < 0.75;
      bool b_near_docking_station = (robot_pos_).norm() < 1.5;
      auto near_dock_msg = std_msgs::msg::Bool();
      near_dock_msg.data = b_near_docking_station;
      near_docking_publisher_->publish(near_dock_msg);
      if (b_near_docking_station) {
        if (b_replan_to_docking_station_) {
          // dock if requested and we are near docking station
          if (b_prepare_to_dock_) {
            if (b_dock_visible_) {
              send_dock_command();
              std::cout << "Docking" << std::endl;
            }
          }

          // we have arrived to the docking station
          b_replan_to_docking_station_ = false;
          b_replan_to_docking_station_sent_ = false;
          std::cout << "Arrived near docking station" << std::endl;
        }
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
              pow(docking_station_pos_map_[0] - msg->poses[msg->poses.size()-1].pose.position.x, 2) +
              pow(docking_station_pos_map_[1] - msg->poses[msg->poses.size()-1].pose.position.y, 2));

      float d_plan = distance_to_goal + distance_goal_to_docking_station;
      float d_available = 6.;     // [FOR TESTING PURPOSES ONLY]
//      float d_available = dist_per_charge_ * (battery_charge_ - min_safe_charge);

      auto d_available_msg = std_msgs::msg::Float32();
      d_available_msg.data = d_available;
      d_available_publisher_->publish(d_available_msg);

      auto d_plan_msg = std_msgs::msg::Float32();
      d_plan_msg.data = d_plan;
      d_plan_publisher_->publish(d_plan_msg);

      // If total path distance is greater than the safe charge distance, then
      // the robot should return to the docking station
      if (d_plan >= d_available) {
        // re-plan and return to docking station
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.pose.position.x = docking_station_pos_map_[0];
        goal.pose.pose.position.y = docking_station_pos_map_[1];
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        replan_to_(goal);

        // set new goal at docking station
        b_replan_to_docking_station_ = true;
        b_prepare_to_dock_ = true;
        auto replan_msg = std_msgs::msg::Bool();
        replan_msg.data = b_replan_to_docking_station_;
        replan_publisher_->publish(replan_msg);
        return;
      }
      auto replan_msg = std_msgs::msg::Bool();
      replan_msg.data = b_replan_to_docking_station_;
      replan_publisher_->publish(replan_msg);
    }

    void battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
//      battery_percentage_ = msg->percentage;
      battery_charge_ = msg->charge;
    }

//    void dock_status_callback(const irobot_create_msgs::msg::DockStatus::SharedPtr msg)
    void dock_status_callback(const amrl_msgs::msg::TurtlebotDockStatus::SharedPtr msg)
    {
      b_dock_visible_ = msg->dock_visible;
      b_is_docked_ = msg->is_docked;
    }

    void cmd_remap_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // Remap cmd_vel to /ut/cmd_vel
      auto cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/ut/cmd_vel", 1);
      cmd_vel_publisher->publish(*msg);
    }

    void replan_to_(const nav2_msgs::action::NavigateToPose::Goal &goal)
    {
      // cancel current plan
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
//      auto cancel_future = nav_to_position_client_->async_cancel_all_goals();
      std::cout << "Canceling current plan" << std::endl;

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&TurtleBot4RechargeMonitorNode::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&TurtleBot4RechargeMonitorNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&TurtleBot4RechargeMonitorNode::result_callback, this, std::placeholders::_1);
      nav_to_pose_client_->async_send_goal(goal, send_goal_options);
//      nav_to_position_client_->async_send_goal(goal, send_goal_options);
      std::cout << "Scheduled send goal" << std::endl;
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal)
    {
      if (!goal) {
        std::cout << "Goal was rejected by server" << std::endl;
      } else {
        std::cout << "Goal accepted by server, waiting for result" << std::endl;
      }
    }

    void feedback_callback(
            const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
            const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
//      std::cout << "Received feedback."<< std::endl;
//      std::cout << "Current pose: " << feedback->current_pose.pose.position.x << ", " << feedback->current_pose.pose.position.y << std::endl;
      std::cout << "Remaining distance: " << feedback->distance_remaining << std::endl;
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Goal succeeded" << std::endl;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          std::cout << "Goal aborted" << std::endl;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          std::cout << "Goal canceled" << std::endl;
          break;
        default:
          std::cout << "Unknown result code" << std::endl;
      }
    }

    void send_dock_command()
    {
//      auto msg = irobot_create_msgs::action::Dock::Goal();
      auto msg = amrl_msgs::action::TurtlebotDock::Goal();
      dock_client_->async_send_goal(msg);
      std::cout << "Sent dock command" << std::endl;
    }

private:
    // Actions
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
//    rclcpp_action::Client<irobot_create_msgs::action::NavigateToPosition>::SharedPtr nav_to_position_client_;
    rclcpp_action::Client<amrl_msgs::action::TurtlebotDock>::SharedPtr dock_client_;
//    rclcpp_action::Client<irobot_create_msgs::action::Dock>::SharedPtr dock_client_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_to_goal_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_subscriber_;
    rclcpp::Subscription<amrl_msgs::msg::TurtlebotDockStatus>::SharedPtr dock_status_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_remap_subscriber_;
//    rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_status_subscriber_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr near_docking_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr d_available_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr d_plan_publisher_;

    // Properties
    Eigen::Vector2f docking_station_pos_; // Docking station position (odom frame)
    Eigen::Vector2f docking_station_pos_map_; // Docking station position (map frame)
    Eigen::Vector2f robot_pos_;           // Robot position (map frame)
    float min_safe_charge;            // Minimum desired battery charge at docking station
    float battery_charge_;                // Current battery charge [Ah]
//    float battery_percentage_;            // Current battery percentage
    float dist_per_charge_;               // Slope of (available) distance (y) vs charge (x) curve
    float dist_offset_;                   // Offset for safety
    bool b_dock_visible_;                   // Flag to check if docking station is visible
    bool b_replan_to_docking_station_;    // Flag to replan to docking stations
    bool b_prepare_to_dock_;              // Flag to prepare to dock
    bool b_is_docked_;                    // Flag to check if robot is docked
    bool b_replan_to_docking_station_sent_;   // Flag to check if replan to docking station has been sent
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
//  Eigen::Vector2f docking_station_pos(-5.5, 23.5);   // if map is provided, dock is fixed
  Eigen::Vector2f docking_station_pos(-0.5, 0.);      // if using slam, dock is in front of robot
  rclcpp::spin(std::make_shared<TurtleBot4RechargeMonitorNode>(docking_station_pos));
  rclcpp::shutdown();
  return 0;
}