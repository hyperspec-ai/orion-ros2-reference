/*
 * MqttBackend.hpp
 *
 *  Created on: 29.05.2019
 *      Author: osboxes
 */

#pragma once
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include "mqtt/async_client.h"
#include "mqttcallback.hpp"
#include <rclcpp/rclcpp.hpp>
namespace config {
class MqttBackend : public virtual mqtt::callback
{
  public:
    /*!
     * @brief Constructor
     */
    MqttBackend();
    /*!
     * @brief Initialize Function for the mqtt backend connection, with call of init also the background processes are started
     * @param [in] host hostname
     * @param [in] veh_name vehicle name/id
     * @param [in] user mqtt user 
     * @param [in] passwd mqtt password
     * @param [in] mqttcb Implementation of mqtt call back interface
     */
    void init(const std::string& host,
              const std::string& veh_name,
              const std::string& user,
              const std::string& passwd,
              MqttCallback* mqttcb);
    /*!
     * @brief Shutdown mqtt connection and end background threads
     */
    void shutdown();

  private:
    //! Small helper function to get the right logger
    inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("CFG_MQB")); }
    /*!
     * @brief Thread publishing monitor information
     */
    void pub_thread();
    /*!
     * @brief Helper thread to execute commands received via mqtt
     */
    void mqtt_connect();
    /*!
     * @brief Call back for lost mqtt broker connection
     * @param [in] cause Cause of connection lost
     */
    void connection_lost(const std::string& cause);
    /*!
     * @brief Call back for delivery completed feedback
     * @param [in] tok Status information for a publich operation
     */
    void delivery_complete(mqtt::delivery_token_ptr tok);
    /*!
     * @brief Call back for delivery message arrived
     * @param [in] msg received message
     */
    void message_arrived(mqtt::const_message_ptr msg);
    /*!
     * @brief mqtt host
     */
    std::string m_host;
    /*!
     * @brief mqtt user
     */
    std::string m_user;
    /*!
     * @brief mqtt password
     */
    std::string m_password;
    /*!
     * @brief vehicle id or name
     */
    std::string m_vehicle_name;
    /*!
     * @brief Refernce to thread object for publish thread
     */
    std::thread* m_thread;

    /*!
     * @brief Internal marker for connection state
     */
    std::atomic<bool> m_connected;
    /*!
     * @brief Interfnal flag used to shutdown all tasks when false
     */
    std::atomic<bool> m_run;
    /*!
     * @brief Pointer to aynchronous mqtt client object (paho)
     */
    mqtt::async_client* m_client;
    /*!
     * Callback for received messages
     **/
    MqttCallback* m_mqttcb;
};
} // namespace config