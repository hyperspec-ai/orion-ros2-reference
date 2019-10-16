#pragma once
#include <string>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
namespace config
{
class NodeCfg
{
   public:
    // Node name
    std::string m_name;
    // Package name
    std::string m_package;
    // Version
    std::string m_version;
    // Node Name
    std::string m_node;
    // Current state
    uint8_t m_current_state;
    // Expected state
    uint8_t m_set_state;
    // Lifecycle Client service for get state
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_client_get_state;
    // Lifecycle Client service for set state
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_client_change_state;
};
}  // namespace config