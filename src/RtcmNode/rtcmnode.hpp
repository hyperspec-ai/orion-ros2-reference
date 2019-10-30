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
namespace rtcm {
class RtcmNode : public rclcpp_lifecycle::LifecycleNode, public ntrip::NtripCallback
{
  public:
    /**
     * Constructor
     * @param option Node options
     */
    explicit RtcmNode(const rclcpp::NodeOptions& options);
    /**
     * Desctuctor
     **/
    virtual ~RtcmNode();

    /**
     * @brief Configuration callback from lifecycle management, requests the node to go to configured state
     * @param state previous state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override;
    /**
     * @brief Configuration callback from lifecycle management, requests the node to go to active state
     * @param state previous state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override;
    /**
     * @brief Configuration callback from lifecycle management, requests the node to go to inactive state
     * @param state previous state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override;
    /**
     * @brief Configuration callback from lifecycle management, requests the node to go to cleanup state
     * @param state previous state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) override;
    /**
     * @brief Configuration callback from lifecycle management, requests the node to go to shut down state
     * @param state previous state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state) override;
    /**
     * @brief Callback for received RTCM data from the ntrip client
     * 
     * @param data RTCM data
     */
    void data_received(std::vector<uint8_t> data) override;

  private:
    /*! RTCM Header Size **/
    const uint16_t HEADER_SIZE = 3;
    /*! RTCM CRC Size **/
    const uint16_t CRC_SIZE = 3;
    /*! RTCM Overhead Size **/
    const uint16_t OVERHEAD_SIZE = HEADER_SIZE + CRC_SIZE;
    /*! RTCM Frame Start indicator **/
    const uint8_t FRAME_START = 0xd3;

    /*! Pointer to instatiated ntrip client */
    ntrip::NtripClient* m_ntrip = nullptr;
    /*! Node Name */
    std::string m_node_name = "";
    /*! Pointer to instanttiated publisher */
    rclcpp_lifecycle::LifecyclePublisher<rtcm_msgs::msg::RtkRTCM>::SharedPtr m_publisher;

    /*! ntrip caster host name or ip  as parameter*/
    rclcpp::Parameter m_host;
    /*! ntrip caster port as parameter*/
    rclcpp::Parameter m_port;
    /*! ntrip caster mount point as parameter*/
    rclcpp::Parameter m_mountpoint;
    /*! ntrip caster user as parameter*/
    rclcpp::Parameter m_user;
    /*! ntrip caster password as parameter*/
    rclcpp::Parameter m_password;
};
} // namespace rtcm
