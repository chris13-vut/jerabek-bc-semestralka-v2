#include "flight_node.hpp"

using namespace std::chrono_literals;

FlightNode::FlightNode() : Node("global_flight_node") {
    mission_ = std::make_unique<MissionManager>();
    // Definice trasy
    mission_->add_waypoint(49.22761148, 16.57039144);
    mission_->add_waypoint(49.22786851, 16.57119060);
    mission_->add_waypoint(49.22702478, 16.57182034);
    mission_->add_waypoint(49.22676566, 16.57102436);

    auto sensor_qos = rclcpp::QoS(10).best_effort();
    
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", sensor_qos,
        std::bind(&FlightNode::gps_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        std::bind(&FlightNode::state_callback, this, std::placeholders::_1));

    alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/rel_alt", sensor_qos,
        std::bind(&FlightNode::alt_callback, this, std::placeholders::_1));

    pos_pub_ = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(
        "/mavros/setpoint_raw/global", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&FlightNode::timer_callback, this));
}

void FlightNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    c_lat_ = msg->latitude;
    c_lon_ = msg->longitude;
    gps_ready_ = true;

    // Pokud ještě nemáme uložen start, ulož ho jako Home
    if (!home_set_) {
        home_lat_ = msg->latitude;
        home_lon_ = msg->longitude;
        home_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Domovská pozice uložena: %.8f, %.8f", home_lat_, home_lon_);
    }
}

void FlightNode::timer_callback() {
    if (!gps_ready_ || !home_set_) return;

    if (current_state_.mode != "GUIDED") {
        drone_->set_mode("GUIDED");
        return;
    } 
    if (!current_state_.armed) {
        drone_->arm(true);
        return;
    }

    static bool takeoff_initiated = false;
    static bool navigation_active = false;
    static bool returning_home = false;

    if (!takeoff_initiated) {
        drone_->takeoff(15.0);
        takeoff_initiated = true;
        return;
    }

    if (!navigation_active) {
        if (c_rel_alt_ > 10.0) {
            navigation_active = true;
            RCLCPP_INFO(this->get_logger(), "Start navigace.");
        } else return;
    }

    // LOGIKA KONCE MISE S NÁVRATEM
    if (mission_->is_mission_finished()) {
        if (!returning_home) {
            RCLCPP_INFO(this->get_logger(), "Trasa proletěna. Vracím se na místo vzletu.");
            returning_home = true;
        }

        // Pokud jsme už blízko místa vzletu (home), přistaneme
        double dist_to_home = mission_->calculate_distance(c_lat_, c_lon_, home_lat_, home_lon_);
        if (dist_to_home < 3.0) {
            RCLCPP_INFO(this->get_logger(), "Dron je nad místem vzletu. Přistávám.");
            drone_->land();
            timer_->cancel();
            return;
        }

        // Cíl je nyní místo vzletu
        mavros_msgs::msg::GlobalPositionTarget msg;
        msg.header.stamp = this->now();
        msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
        msg.type_mask = 0xFFF8; 
        msg.latitude = home_lat_;
        msg.longitude = home_lon_;
        msg.altitude = 15.0;
        pos_pub_->publish(msg);
        return;
    }

    // Navigace po waypointech
    if (mission_->is_at_destination(c_lat_, c_lon_, 3.0)) {
        RCLCPP_INFO(this->get_logger(), "Bod dosažen.");
        mission_->next_waypoint();
    }

    auto target = mission_->get_current_target();
    mavros_msgs::msg::GlobalPositionTarget msg;
    msg.header.stamp = this->now();
    msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    msg.type_mask = 0xFFF8; 
    msg.latitude = target.lat;
    msg.longitude = target.lon;
    msg.altitude = 15.0;
    pos_pub_->publish(msg);
}

// Zbytek funkcí 
void FlightNode::init() { drone_ = std::make_shared<DroneInterface>(shared_from_this()); }
void FlightNode::state_callback(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
void FlightNode::alt_callback(const std_msgs::msg::Float64::SharedPtr msg) { c_rel_alt_ = msg->data; }
