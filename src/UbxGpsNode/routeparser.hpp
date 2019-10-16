#pragma once
#include <string>
#include "json/include/nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
class RouteParser
{
  public:
    RouteParser(const std::string& routes_json, const std::string& node_name);
    std::string get_topic_by_endpoint(std::string);

  private:
    nlohmann::json m_routes;
    std::string m_node_name;
    // Helper function for logging
    static inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("ROUTE")); }
};