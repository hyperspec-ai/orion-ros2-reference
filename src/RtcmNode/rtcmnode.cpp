#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iterator>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rtcm_msgs/msg/rtk_rtcm.hpp"
#include "rtcmnode.hpp"
namespace rtcm
{
std::string get_node_name(std::vector<rclcpp::Parameter> parameters)
{
    std::string ret_val = "rtcm_node";
    for (auto const& parameter : parameters)
    {
        if (parameter.get_name() == "node_name")
        {
            ret_val = parameter.as_string();
        }
    }
    return ret_val;
}

RtcmNode::RtcmNode(const rclcpp::NodeOptions& options)
    : LifecycleNode(get_node_name(options.parameter_overrides()), options)
{
    m_node_name = get_node_name(options.parameter_overrides());
    RCLCPP_INFO(this->get_logger(), "RTCM Node instantiated with name:%s", m_node_name.c_str());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RtcmNode::on_configure(
    const rclcpp_lifecycle::State&)
{
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret_val =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    try
    {
        m_publisher =
            this->create_publisher<rtcm_msgs::msg::RtkRTCM>("/" + m_node_name + "/rtcm_data", 10);
        m_host = this->get_parameter("ntrip_host");
        m_port = this->get_parameter("ntrip_port");
        m_mountpoint = this->get_parameter("ntrip_mountpoint");
        m_user = this->get_parameter("ntrip_user");
        m_password = this->get_parameter("ntrip_password");
        RCLCPP_INFO(this->get_logger(), "Configuration done");
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Error while reading config parameters:%s", e.what());
        ret_val = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return (ret_val);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RtcmNode::on_activate(
    const rclcpp_lifecycle::State&)
{
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret_val =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    if (m_ntrip == nullptr)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Try Starting ntrip client");
            m_ntrip = new ntrip::NtripClient(m_host.as_string(), m_port.as_int(),
                                             m_mountpoint.as_string(), this, m_user.as_string(),
                                             m_password.as_string());
        }
        catch (const std::exception& e)
        {
            ret_val =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }
    m_publisher->on_activate();
    return (ret_val);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RtcmNode::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Publishing deactivated");
    m_publisher->on_deactivate();
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RtcmNode::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    if (m_ntrip != nullptr)
    {
        m_ntrip->shutdown();
        delete (m_ntrip);
        m_ntrip = nullptr;
    }
    m_publisher.reset();
    RCLCPP_INFO(this->get_logger(), "Cleaned up Node");
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RtcmNode::on_shutdown(
    const rclcpp_lifecycle::State&)
{
    if (m_ntrip != nullptr)
    {
        m_ntrip->shutdown();
        delete (m_ntrip);
        m_ntrip = nullptr;
    }
    m_publisher.reset();
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}
void RtcmNode::data_received(std::vector<uint8_t> data)
{
    RCLCPP_INFO(this->get_logger(), "Data Reveived");
    uint16_t pos = 0;
    while (pos < data.size())
    {
        while ((pos < data.size()) && (data[pos] != FRAME_START))
        {
            pos++;
        }
        if (data[pos] == FRAME_START)
        {
            uint16_t pkt_len = (((uint16_t)(data[pos + 1] & 0x3)) << 8) + data[pos + 2];
            uint16_t overall_length = pkt_len + OVERHEAD_SIZE;
            /*Check size on plausibility*/
            if ((pkt_len + OVERHEAD_SIZE) <= (data.size() - pos))
            {
                RCLCPP_INFO(this->get_logger(), "Length plaussible, start Pos: %d, length: %d", pos,
                            overall_length);
                auto msg = rtcm_msgs::msg::RtkRTCM();
                msg.rtcm_data.resize(overall_length);
                memcpy(&msg.rtcm_data[0], &data[pos], overall_length);
                msg.rtcm_length = overall_length;
                msg.header.stamp = rclcpp::Clock().now();
                msg.header.frame_id = "rtcm_frames";
                m_publisher->publish(msg);
                pos += overall_length;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                            "Length not plaussible, start Pos: %d, length: %d, buffer size", pos,
                            overall_length, data.size());
                break;
            }
        }
    }
}
RtcmNode::~RtcmNode()
{
    if (m_ntrip != nullptr)
    {
        m_ntrip->shutdown();
        delete (m_ntrip);
        m_ntrip = nullptr;
    }
}
}  // namespace rtcm
RCLCPP_COMPONENTS_REGISTER_NODE(rtcm::RtcmNode)