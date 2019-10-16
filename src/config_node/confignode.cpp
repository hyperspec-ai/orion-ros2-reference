#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iterator>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "class_loader/class_loader.hpp"
#include "confignode.hpp"
#include "rclcpp_components/node_factory.hpp"

namespace config
{
ConfigNode::ConfigNode(const rclcpp::NodeOptions& options) : Node("config_node", options)
{
    m_running = true;
    if (!this->has_parameter("path"))
    {
        this->set_parameter({"path", "/etc/"});
        RCLCPP_WARN(this->get_logger(),
                    "Path parameter not set propably no configuration will be found");
    }
    m_path = this->get_parameter("path");
    RCLCPP_INFO(this->get_logger(), "Start executor thread");
    m_executor = new std::thread([this] { worker_thread(); });
}

void ConfigNode::worker_thread()
{
    m_config_handler = new ConfigurationHandler(m_path.as_string(), this);
    if (m_config_handler->base_config_loaded())
    {
        RCLCPP_INFO(this->get_logger(), "Base Config Loaded");
        if (m_config_handler->system_config_loaded())
        {
            RCLCPP_INFO(this->get_logger(), "System Config Loaded");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Base Config Failed");
    }

    while (m_running)
    {
        /*Todo Monitoring*/
        sleep(1);
    }
}

ConfigNode::~ConfigNode()
{
    m_running = false;
}
}  // namespace config
RCLCPP_COMPONENTS_REGISTER_NODE(config::ConfigNode)
