#include "mission_manager.hpp"

MissionManager::MissionManager() {}

void MissionManager::add_waypoint(double lat, double lon) {
    waypoints_.push_back({lat, lon});
}

Waypoint MissionManager::get_current_target() const {
    if (current_wp_index_ < waypoints_.size()) return waypoints_[current_wp_index_];
    return {0, 0};
}

bool MissionManager::next_waypoint() {
    if (current_wp_index_ < waypoints_.size()) {
        current_wp_index_++;
        return true;
    }
    return false;
}

bool MissionManager::is_mission_finished() const {
    return current_wp_index_ >= waypoints_.size();
}

bool MissionManager::is_at_destination(double cur_lat, double cur_lon, double tolerance_m) {
    Waypoint target = get_current_target();
    return calculate_distance(cur_lat, cur_lon, target.lat, target.lon) < tolerance_m;
}

double MissionManager::calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon/2) * std::sin(dlon/2);
    return 2 * std::atan2(std::sqrt(a), std::sqrt(1-a)) * 6371000.0;
}
