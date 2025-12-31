#ifndef FLIGHT_NODE_HPP_
#define FLIGHT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "drone_interface.hpp"
#include "mission_manager.hpp"

class FlightNode : public rclcpp::Node {
public:
    FlightNode();
    void init();
    bool has_gps() const { return gps_ready_; }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void alt_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void timer_callback();

    std::shared_ptr<DroneInterface> drone_;
    std::unique_ptr<MissionManager> mission_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double c_lat_{0}, c_lon_{0}, c_rel_alt_{0};
    double home_lat_{0}, home_lon_{0}; 
    bool gps_ready_{false};
    bool home_set_{false};            
    mavros_msgs::msg::State current_state_;
};

#endif
