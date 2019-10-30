#pragma once
#include <string>
#include <vector>
#include "executionhandler.hpp"
#include "lifecyclemanager.hpp"
#include "mqttbackend.hpp"
#include "mqttcallback.hpp"
#include "nodecfg.hpp"
#include "persistencehandler.hpp"
#include "rclcpp/rclcpp.hpp"
namespace config {
/*!
 * Responsible for reading and executing the Base and system configuration
 **/
class ConfigurationHandler : public MqttCallback
{
  public:
    /**
     * Constructor
     *
     * @param config_path path to working directory containing base conig and dirs
     * for libs etc.
     * @param node Pointer to config node, neccesary to create client services for
     * life cycle management
     */
    explicit ConfigurationHandler(const std::string& config_path, rclcpp::Node* node);
    /**
     * Getter for base config load status
     * @return true in case base_config.json was sucsessfully loaded
     */
    bool base_config_loaded();
    /**
     * Getter for system config load status
     * @return true in case system_config.json was sucsessfully loaded
     */
    bool system_config_loaded();
    /**
     * Function loading the system configuration and instantiating the neccesary nodes
     * @return true in case loading/inmstantiation was successfulle
     */
    bool load_system_configuration();
    /**
     * Function unloading the system configuration and clean up execution/node enviroment
     * @return true in case unload was succesfull
     */
    bool unload_system_configuration();
    /**
     * Getter for execution location (INT_X_1 etc.)
     * @return String representing the execution location
     */
    std::string get_execution_location();

    /**
     * @brief Mqtt call back implementation
     * 
     * @param msg received message
     */
    void message_recieved(mqtt::const_message_ptr msg) override;

    /*System States*/
    enum system_state_t
    {
        //! Stopped
        STATE_STOPED = 0,
        //! Starting in progress
        STATE_STARTING = 1,
        //! System running
        STATE_STARTED = 2,
        //! System Shutting down
        STATE_STOPING = 3,
        //! System in error
        STATE_ERROR = 4
    };

  private:
    //! System state
    system_state_t m_system_state = STATE_STOPED;
    //! true in case the base config was sucessfully loaded
    bool m_base_config_loaded = false;
    //! true in case system config/instantiation was succcesull
    bool m_system_config_loaded = false;
    //! Execution location by base config (INT_X_1 etc.)
    std::string m_exec_location = "";
    //! Path to base config
    std::string m_path = "";
    //! Vehicle name/id
    std::string m_vehicle = "";
    //! Mqtt Host from base config
    std::string m_mqtt_host = "";
    //! Mqtt user from base config
    std::string m_mqtt_user = "";
    //! Mqtt password from base config
    std::string m_mqtt_password = "";
    //! Flag if system config should be loaded on startup
    bool m_system_config_av = false;
    //! Path to working directory from base config
    std::string m_config_path = "";
    //! Life cycle manager instance
    LifecycleManager m_lifecycle_manager;
    //! Execution handler instance
    ExecutionHandler m_exec_handler;
    //! Mqtt backend instance
    MqttBackend m_mqtt_backend;
    //! Pointer to persistence handler
    PersistenceHandler* m_persistence_handler;
    //! Small helper to set the correct logger
    inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("CFG_CGH")); }
};
} // namespace config