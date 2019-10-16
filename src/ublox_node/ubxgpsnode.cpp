#include <unistd.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "ubxgpsnode.hpp"
namespace ubx_node {
std::string get_node_name(std::vector<rclcpp::Parameter> parameters)
{
    std::string ret_val = "ubx_node";
    for(auto const& parameter : parameters)
    {
        if(parameter.get_name() == "node_name")
        {
            ret_val = parameter.as_string();
        }
    }
    return ret_val;
}

UbxGpsNode::UbxGpsNode(const rclcpp::NodeOptions& options) :
    LifecycleNode(get_node_name(options.parameter_overrides()), options),
    m_node_name(get_node_name(options.parameter_overrides())),
    m_status_msg(this),
    m_loc_msg(this),
    m_eoe_msg(this),
    m_pvt_msg(this),
    m_running(true),
    m_thread_running(true),
    m_eoe_count(0)

{
    // Init atomic members
    // m_running = true;
    // m_thread_running = true;
    // m_eoe_count = 0;
    // m_node_name = get_node_name(options.parameter_overrides());
    RCLCPP_INFO(this->get_logger(), "UBX Node instantiated with name:%s", m_node_name.c_str());
}
UbxGpsNode::~UbxGpsNode()
{
    de_init_sensor();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UbxGpsNode::
    on_configure(const rclcpp_lifecycle::State&)
{
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret_val =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    try
    {
        // Get serial port, standard cofig should be /dev/ttyUB0
        m_port = this->get_parameter("serial_port");
        // get sample timing in ms from config
        m_sample_time = this->get_parameter("sampling_time");
        m_rtcm_topic = this->get_parameter("rtcm_topic");
        // Create new publisher
        m_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/" + m_node_name + "/gnss_fix", 10);
        RCLCPP_INFO(this->get_logger(), "Configuration done");
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Error while reading config parameters:%s", e.what());
        ret_val =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return (ret_val);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UbxGpsNode::on_activate(
    const rclcpp_lifecycle::State&)
{
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret_val =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    if(m_ubx_ser == nullptr)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Try Starting ubx driver");

            /*Check if correction service is avilable*/
            if(m_rtcm_topic.as_string() != "")
            {
                m_rtcm_subscriber = this->create_subscription<rtcm_msgs::msg::RtkRTCM>(
                    m_rtcm_topic.as_string(), 10, [this](rtcm_msgs::msg::RtkRTCM::UniquePtr msg) {
                        this->m_ubx_ser->add_rtcm_message(msg->rtcm_data);
                        RCLCPP_INFO(this->get_logger(), "Correction Data Received");
                    });
            }

            init_sensor();
        }
        catch(const std::exception& e)
        {
            ret_val =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }
    m_publisher->on_activate();
    return (ret_val);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UbxGpsNode::
    on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Publishing deactivated");
    m_publisher->on_deactivate();
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UbxGpsNode::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Cleaned up Node");
    de_init_sensor();
    m_publisher.reset();
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UbxGpsNode::on_shutdown(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Shutdown Node");
    de_init_sensor();
    m_publisher.reset();
    return (rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void UbxGpsNode::monitor_thread()
{
    RCLCPP_INFO(this->get_logger(), "Monitoring started");

    // time out time is 4 times the set smaple time in us (time in ms)
    uint32_t sleep_time_us = ((uint32_t)m_sample_time.as_int()) * 4000;
    // Give some time to settle configuration (5s)
    sleep(5);

    /// While sensor is not deinited run
    while(m_thread_running)
    {
        // Check only in case sensor is active
        if(m_running)
        {
            /**
             * Wait the defined time
             */
            usleep(sleep_time_us);
            /**
             * Check of end of epoch message was received
             */
            if(m_eoe_count == m_eoe_count_old)
            {
            }
            m_eoe_count_old = m_eoe_count;

            if(m_rtk_corection_lost)
            {
                uint32_t delta = time(nullptr) - m_rtk_corection_timeout;
                if(delta == 30)
                {
                }
            }
        }
    }
}
uint16_t UbxGpsNode::init_sensor()
{
    assert(m_ubx_ser == nullptr);
    m_running = true;
    m_thread_running = true;
    m_ubx_ser = new UbxSerialHandler(m_port.as_string());
    // Setup and enque configuration messages

    {
        UbxUsbProtCfg* usb_prot_cfg = new UbxUsbProtCfg();
        usb_prot_cfg->set_nmea_output_mode(UbxUsbProtCfg::PROT_OFF);
        usb_prot_cfg->set_rtcm_output_mode(UbxUsbProtCfg::PROT_OFF);
        usb_prot_cfg->set_ubx_output_mode(UbxUsbProtCfg::PROT_ON);

        m_ubx_ser->enqueue_ubx_cfg_message(usb_prot_cfg);
    }

    {
        UbxUsbMsgCfg* usb_msg_cfg = new UbxUsbMsgCfg();
        usb_msg_cfg->set_highprecission_msg(1);
        usb_msg_cfg->set_EOE_msg(1);
        usb_msg_cfg->set_status_msg(1);
        usb_msg_cfg->set_pvg_msg(1);

        m_ubx_ser->enqueue_ubx_cfg_message(usb_msg_cfg);
    }

    {
        UbxNavRateCfg* nav_rate_cfg = new UbxNavRateCfg();
        nav_rate_cfg->set_meas_rate_ms(m_sample_time.as_int());
        nav_rate_cfg->set_rate_nav(1);
        nav_rate_cfg->set_rate_time_ref(UbxNavRateCfg::TIMEREF_GPS);

        m_ubx_ser->enqueue_ubx_cfg_message(nav_rate_cfg);
    }

    // Register recive messages
    m_ubx_ser->register_receive_msg(&m_loc_msg);
    m_ubx_ser->register_receive_msg(&m_status_msg);
    m_ubx_ser->register_receive_msg(&m_eoe_msg);
    m_ubx_ser->register_receive_msg(&m_pvt_msg);
    // Init comport and start reception
    if(!m_ubx_ser->init_comport())
    {
        RCLCPP_ERROR(this->get_logger(), "COM Port Init failed");
    }
    // Start monitoring
    m_monitor_thread = new std::thread([this] { monitor_thread(); });
    return (0);
}

uint16_t UbxGpsNode::de_init_sensor()
{
    if(m_ubx_ser != nullptr)
    {
        m_thread_running = false;
        m_running = false;
        m_monitor_thread->join();
        RCLCPP_INFO(this->get_logger(), "Closing Port");
        m_ubx_ser->close_port();
        RCLCPP_INFO(this->get_logger(), "Destructing m_ubx_ser");
        delete(m_ubx_ser);
        RCLCPP_INFO(this->get_logger(), "Clean up monitor thread");
        delete(m_monitor_thread);
        m_ubx_ser = nullptr;
    }
    return (0);
}

void UbxGpsNode::message_received(UbxMessage::Identifier id, const UbxMessage* msg)
{
    if(m_running)
    {
        if(id == m_loc_msg.get_identifier())
        {
            UbxNavHighPresGeo* loc_msg = (UbxNavHighPresGeo*)msg;
            m_acc_h = loc_msg->get_horicontal_acc();
            m_acc_v = loc_msg->get_vertical_acc();
            m_lon = loc_msg->get_lon();
            m_lat = loc_msg->get_lat();
            m_height = loc_msg->get_height();
        }
        else if(id == m_status_msg.get_identifier())
        {
            UbxNavStatus* stat_msg = (UbxNavStatus*)msg;
            uint32_t fix = stat_msg->get_gps_fix();
            uint8_t flags1 = stat_msg->get_nav_status_flags1();
            switch(fix)
            {
                case UbxNavStatus::GPS_FIX_NO_FIX:
                    m_rtk_corection_lost = false;
                    m_gnns_mode = -1;
                    break;
                case UbxNavStatus::GPS_FIX_2D:
                    m_rtk_corection_lost = false;
                    m_gnns_mode = 0;
                    break;
                case UbxNavStatus::GPS_FIX_3D:
                    if((flags1 & 0x0f) == 0x0F)
                    {
                        m_gnns_mode = 2;
                        m_rtk_corection_lost = false;
                    }
                    else
                    {
                        m_gnns_mode = 0;
                        if(m_rtk_corection_lost == false)
                        {
                            m_rtk_corection_timeout = time(nullptr);
                        }
                        m_rtk_corection_lost = true;
                    }
                    break;
                default: m_gnns_mode = -1; break;
            }
        }
        else if(id == m_pvt_msg.get_identifier())
        {
            UbxNavPvt* pvt_msg = (UbxNavPvt*)msg;
            m_velocity = pvt_msg->get_ground_speed();
            m_velocity_acc = pvt_msg->get_ground_speed_acc();
            m_heading = pvt_msg->get_heading();
            m_heading_acc = pvt_msg->get_heading_acc();
        }
        else if(id == m_eoe_msg.get_identifier())
        {
            // spdlog::trace("EOE Received");
            m_eoe_count++;
            auto message = sensor_msgs::msg::NavSatFix();
            message.latitude = m_lat;
            message.longitude = m_lon;
            message.altitude = m_height;
            message.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            message.position_covariance[0] = m_acc_h;
            message.position_covariance[4] = m_acc_h;
            message.position_covariance[8] = m_acc_v;
            message.status.status = m_gnns_mode;
            message.status.service = message.status.SERVICE_GALILEO |
                message.status.SERVICE_COMPASS | message.status.SERVICE_GLONASS |
                message.status.SERVICE_GPS;
            message.header.stamp = rclcpp::Clock().now();
            message.header.frame_id = "gps";

            m_publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "End of Epoch received send ROS2 message");
            m_frame++;
        }
    }
}
} // namespace ubx_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ubx_node::UbxGpsNode)
