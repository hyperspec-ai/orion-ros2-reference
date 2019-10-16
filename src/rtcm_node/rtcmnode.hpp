#pragma once
#include <atomic>
#include <thread>
#include "ntripclient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rtcm_msgs/msg/rtk_rtcm.hpp"
/**
 * ROS2 Node intetface for rtmc correction data
 * TODO: External config
 **/
namespace rtcm
{
class RtcmNode : public rclcpp_lifecycle::LifecycleNode, public ntrip::NtripCallback
{
   public:
    /**
     * Constructor
     */
    explicit RtcmNode(const rclcpp::NodeOptions& options);
    /**
     * Desctuctor
     **/
    virtual ~RtcmNode();
    /*Lifecylce Nodes declarations*/
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state) override;

    void data_received(std::vector<uint8_t> data) override;

   private:
    ntrip::NtripClient* m_ntrip = nullptr;
    std::string m_node_name = "";
    rclcpp_lifecycle::LifecyclePublisher<rtcm_msgs::msg::RtkRTCM>::SharedPtr m_publisher;
    const uint16_t HEADER_SIZE = 3;
    const uint16_t CRC_SIZE = 3;
    const uint16_t OVERHEAD_SIZE = HEADER_SIZE + CRC_SIZE;
    const uint8_t FRAME_START = 0xd3;
    rclcpp::Parameter m_host;
    rclcpp::Parameter m_port;
    rclcpp::Parameter m_mountpoint;
    rclcpp::Parameter m_user;
    rclcpp::Parameter m_password;
};
}  // namespace rtcm
