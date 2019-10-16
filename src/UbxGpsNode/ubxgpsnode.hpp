#pragma once
#include <atomic>
#include <thread>
#include "messages/ubxnaveoe.hpp"
#include "messages/ubxnavhighpresgeo.hpp"
#include "messages/ubxnavpvt.hpp"
#include "messages/ubxnavratecfg.hpp"
#include "messages/ubxnavstatus.hpp"
#include "messages/ubxusbmsgcfg.hpp"
#include "messages/ubxusbprotcfg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rtcm_msgs/msg/rtk_rtcm.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ubxcbinterface.hpp"
#include "ubxserialhandler.hpp"

/**
 * ROS2 Node intetface for ublox gps receiver
 * TODO: External config
 **/
namespace ubx_node {
class UbxGpsNode : public UbxCbInterface, public rclcpp_lifecycle::LifecycleNode
{
  public:
    /**
     * Constructor
     */
    UbxGpsNode(const rclcpp::NodeOptions& options);
    virtual ~UbxGpsNode();
    /**
     * Start sensor
     */
    uint16_t init_sensor();
    /**
     * Stop sensor
     */
    uint16_t de_init_sensor();
    /**
     * Callback for ubx messages
     * NavSatFix is published after receiveing end of epoch message
     */
    virtual void message_received(UbxMessage::Identifier id, const UbxMessage* msg) override;

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

  private:
    /**
     * Monitoring thread, checking on timeouts of recived messages
     */
    void monitor_thread();
    /// ROS2 publisher of type NavSatFix
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_publisher;
    rclcpp::Subscription<rtcm_msgs::msg::RtkRTCM>::SharedPtr m_rtcm_subscriber;
    /// Threading object for moinitor thread
    std::thread* m_monitor_thread;
    /// Comport path
    rclcpp::Parameter m_port;
    /// Sample time in ms
    rclcpp::Parameter m_sample_time;
    /// RTCM Topic
    std::string m_rtcm_topic;
    /// Node Name
    std::string m_node_name;
    /// Received information to be cached until eoe is received
    double m_lat = 0;
    double m_lon = 0;
    double m_acc_h = 0;
    double m_acc_v = 0;
    double m_height = 0;
    double m_velocity = 0;
    double m_velocity_acc = 0;
    double m_heading = 0;
    double m_heading_acc = 0;
    int8_t m_gnns_mode = 0;

    /// Instance of the serial handler used
    UbxSerialHandler* m_ubx_ser = nullptr;
    /// Message definitions
    UbxNavStatus m_status_msg;
    UbxNavHighPresGeo m_loc_msg;
    UbxNavEoe m_eoe_msg;
    UbxNavPvt m_pvt_msg;
    /// Thread control variables
    std::atomic<bool> m_running;
    std::atomic<bool> m_thread_running;
    /// RTK Status information
    std::atomic<bool> m_rtk_corection_lost;
    std::atomic<uint32_t> m_rtk_corection_timeout;
    std::atomic<uint64_t> m_eoe_count;
    uint64_t m_eoe_count_old;
    uint16_t m_frame = 0;
};
} // namespace ubx_node
