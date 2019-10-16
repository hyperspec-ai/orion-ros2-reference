/*
 * MqttBackend.cpp
 *
 *  Created on: 29.05.2019
 *      Author: osboxes
 */

#include <sys/stat.h>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "mqttbackend.hpp"
#include "mqttcallback.hpp"
namespace config
{
MqttBackend::MqttBackend() {}
void MqttBackend::connection_lost(const std::string& cause)
{
    m_connected = false;
}

void MqttBackend::delivery_complete(mqtt::delivery_token_ptr tok) {}

void MqttBackend::init(const std::string& host, const std::string& veh_name,
                       const std::string& user, const std::string& passwd, MqttCallback* mqttcb)
{
    assert(mqttcb != nullptr);
    m_vehicle_name = veh_name;
    m_mqttcb = mqttcb;
    m_host = host;
    m_run = true;
    m_user = user;
    m_password = passwd;
    m_thread = new std::thread([this] { pub_thread(); });
}
void MqttBackend::shutdown()
{
    m_run = false;
}

void MqttBackend::pub_thread()
{
    m_client = new mqtt::async_client(m_host, m_vehicle_name);
    m_client->set_callback(*this);
    mqtt_connect();
    while (m_run)
    {
        usleep(500000);
    }
    // Wait a short moment before reconnect
    usleep(500000);
}

void MqttBackend::mqtt_connect()
{
    mqtt::connect_options conopts(m_user.c_str(), m_password.c_str());
    bool state = false;
    RCLCPP_INFO(this->get_logger(), "Try to connect to mqtt broker");

    conopts.set_automatic_reconnect(true);
    mqtt::message willmsg("/" + m_vehicle_name + "/state", "{\"state\":0}", 1, true);
    mqtt::will_options will(willmsg);
    conopts.set_will(will);

    while ((state == false) && (m_run))
    {
        mqtt::token_ptr conntok = m_client->connect(conopts);
        state = conntok->wait_for(10000);
        if (state == false)
        {
            RCLCPP_ERROR(this->get_logger(), "Connection to mqtt broker failed");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Connect succesfull");
            std::string topic = "/" + m_vehicle_name + "/command";
            m_client->subscribe(topic, 0);
            std::string topic2 = "/" + m_vehicle_name + "/config";
            m_client->subscribe(topic2, 0);
            m_connected = true;
        }
    }
}
void MqttBackend::message_arrived(mqtt::const_message_ptr msg)
{
    RCLCPP_INFO(this->get_logger(), "MQTT Received Topic %s \r\n Message: %s",
                msg->get_topic().c_str(), msg->to_string().c_str());
    m_mqttcb->message_recieved(msg);
}

}  // namespace config