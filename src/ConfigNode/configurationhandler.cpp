#include <unistd.h>
#include <fstream>
#include "configurationhandler.hpp"
#include "json/include/nlohmann/json.hpp"
namespace config {
ConfigurationHandler::ConfigurationHandler(const std::string& config_path, rclcpp::Node* node) :
    m_config_path(config_path),
    m_lifecycle_manager(node)
{
    try
    {
        std::string base_config_path = m_config_path + "/base_config.json";
        std::ifstream json_file(base_config_path);

        RCLCPP_INFO(
            this->get_logger(), "Start reading base configuration %s", base_config_path.c_str());
        nlohmann::json json_object;
        json_file >> json_object;
        RCLCPP_INFO(this->get_logger(), "JSON Succesfully loaded");
        /*Check if all neccesary config items of the base config are available*/
        bool vehicle_conf = json_object.contains(nlohmann::json("vehicle"));
        bool exec_location_conf = json_object.contains(nlohmann::json("execution_location"));
        bool mqtt_conf = json_object.contains(nlohmann::json("mqtt"));
        bool system_conf = json_object.contains(nlohmann::json("system_config"));

        if(vehicle_conf && exec_location_conf && mqtt_conf && system_conf)
        {
            /*Load all neccesary config values*/
            m_base_config_loaded = true;
            if(json_object["vehicle"].is_string())
            {
                m_vehicle = json_object["vehicle"].get<std::string>();
                RCLCPP_INFO(this->get_logger(), "Vehicle: %s", m_vehicle.c_str());
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(json_object["execution_location"].is_string())
            {
                m_exec_location = json_object["execution_location"].get<std::string>();
                RCLCPP_INFO(
                    this->get_logger(), "Execution Location: %s", m_exec_location.c_str());
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(json_object["system_config"].is_boolean())
            {
                m_system_config_av = json_object["system_config"].get<bool>();
                RCLCPP_INFO(this->get_logger(), "Load System Config: %d", m_system_config_av);
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(json_object["path"].is_string())
            {
                m_path = json_object["path"].get<std::string>();
                RCLCPP_INFO(this->get_logger(), "Path: %s", m_path.c_str());
            }
            else
            {
                m_base_config_loaded = false;
            }
            nlohmann::json mqtt_json = json_object["mqtt"];
            if(mqtt_json["host"].is_string())
            {
                m_mqtt_host = mqtt_json["host"].get<std::string>();
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(mqtt_json["user"].is_string())
            {
                m_mqtt_user = mqtt_json["user"].get<std::string>();
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(mqtt_json["password"].is_string())
            {
                m_mqtt_password = mqtt_json["password"].get<std::string>();
            }
            else
            {
                m_base_config_loaded = false;
            }
            if(m_base_config_loaded)
            {
                m_mqtt_backend.init(m_mqtt_host, m_vehicle, m_mqtt_user, m_mqtt_password, this);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Base Configuration errenous");
                m_system_state = STATE_ERROR;
            }
            // Instantiate persistence handler, careing about local node repository and if neccesary
            // downloads
            m_persistence_handler = new PersistenceHandler(m_path, m_vehicle);
            /*If system config is available/should be executed*/
            if(m_system_config_av)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Base Configuration loaded and System Config is available");
                m_system_state = STATE_STARTING;
                if(load_system_configuration())
                {
                    /*Start all executor threads*/
                    m_exec_handler.start_threads();
                    /*wait some time to ensrure that is all up and running*/
                    sleep(5);
                    /*Go to active over configured state*/
                    m_lifecycle_manager.set_all_configured();
                    m_lifecycle_manager.set_all_active();
                    /*Set System state to started, TODO: Check return values and lifecycle states
                     * actively*/
                    m_system_state = STATE_STARTED;
                }
                else
                {
                    /*system configuration load failed*/
                    RCLCPP_ERROR(this->get_logger(), "System Configuration failed");
                    m_system_state = STATE_ERROR;
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Base Configuration failed %s", e.what());
        m_base_config_loaded = false;
    }
}

void ConfigurationHandler::message_recieved(mqtt::const_message_ptr msg)
{
    /**
     * Get topic, curently there are two topics supported
     * /<vehicle_name>/command --> Commands to change the runtime state
     * /<vehicle_name>/config --> to send a new system configuration to the vehicle
     **/
    std::string topic = msg->get_topic();
    if(topic.compare("/" + m_vehicle + "/command") == 0)
    {
        /**
         * Suported Commands:
         * - start: Starts the system configuration stored in system_config.json
         * - stop: shutdown the whole system
         * - reporeset (TODO): reset repository storage and remove all libs (all artifacts will be
         *downloaded again)
         **/
        RCLCPP_INFO(this->get_logger(), "Command Received");
        if(msg->to_string().compare("start") == 0)
        {
            if(m_system_state == STATE_STOPED)
            {
                if(load_system_configuration())
                {
                    m_exec_handler.start_threads();
                }
                else
                {
                    m_system_state = STATE_ERROR;
                }

                sleep(5);
                m_lifecycle_manager.set_all_configured();
                m_lifecycle_manager.set_all_active();
                m_system_state = STATE_STARTED;
            }
        }
        else if(msg->to_string().compare("stop") == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Command Received: Stop");
            if(m_system_state == STATE_STARTED)
            {
                unload_system_configuration();
            }
        }
        else if(msg->to_string().compare("reporeset") == 0)
        {
            /* code */
        }
    }
    if(topic.compare("/" + m_vehicle + "/config") == 0)
    {
        /**
         * If a valid json config is received it will be stored as system_config.json
         **/
        bool valid_json = true;
        bool correct_vehicle = true;
        try
        {
            /**Check if received config is a valid json**/
            auto json_object = nlohmann::json::parse(msg->to_string());
            if(!(json_object["vehicle"].get<std::string>().compare(m_vehicle)))
            {
                correct_vehicle = false;
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "Config is not a valid json, not saved");
            valid_json = false;
        }
        if(valid_json && correct_vehicle)
        {
            std::string system_config_file = m_path + "/system_config.json";
            std::ofstream out(system_config_file);
            out << msg->to_string();
            out.close();
        }
    }
}

bool ConfigurationHandler::unload_system_configuration()
{
    /**
     * Cleanup System
     **/
    bool ret_val = false;
    m_system_state = STATE_STOPING;
    RCLCPP_INFO(this->get_logger(), "Shutting down all nodes via lifecycle management");
    bool da_status = m_lifecycle_manager.set_all_deactive();
    bool sd_status = m_lifecycle_manager.set_all_shutdown();
    RCLCPP_INFO(this->get_logger(), "Shutting down completed, cleaning up execution contexts");
    // Stops and cleanup all nodes / executions
    if(da_status && sd_status)
    {
        m_exec_handler.cleanup();
        m_system_state = STATE_STOPED;
        ret_val = true;
    }
    return (ret_val);
}
bool ConfigurationHandler::load_system_configuration()
{
    bool ret_val = true;
    try
    {
        /**
         * Load and parse system_config.json
         **/

        std::string system_config_file = m_path + "/system_config.json";
        std::ifstream json_file(system_config_file);
        RCLCPP_INFO(this->get_logger(), "Load System Config: %s", system_config_file.c_str());
        nlohmann::json json_object;
        json_file >> json_object;
        RCLCPP_INFO(this->get_logger(), "System Configs %s", json_object.dump().c_str());
        // Check if the vehicle entry fits to the vehicle name in the base config
        if(json_object["vehicle"].is_string())
        {
            if(json_object["vehicle"].get<std::string>().compare(m_vehicle) != 0)
            {
                ret_val = false;
            }
        }
        else
        {
            ret_val = false;
        }
        if(ret_val)
        {
            RCLCPP_INFO(
                this->get_logger(), "Loading System Config for vehicle %s", m_vehicle.c_str());
            // Iterate through message lib entries in the config file, must be done before loading
            // nodes as they might depend on this libs
            nlohmann::json msg_routes_json = json_object["routes"];
            RCLCPP_INFO(this->get_logger(), "Routes Config: %s", msg_routes_json.dump().c_str());
            nlohmann::json msg_libs_json = json_object["msg_libs"];
            for(auto msg_lib_json : msg_libs_json)
            {
                std::string execution_location =
                    msg_lib_json["execution_location"].get<std::string>();
                if(execution_location.compare(m_exec_location) == 0)
                {
                    /*Load all neccesary parameters for persistence management/download*/
                    std::string name = msg_lib_json["name"].get<std::string>();
                    std::string package = msg_lib_json["package"].get<std::string>();
                    std::string messages = msg_lib_json["message"].get<std::string>();
                    std::string version = msg_lib_json["version"].get<std::string>();
                    std::string location = msg_lib_json["package_location"].get<std::string>();
                    std::string finger_print = msg_lib_json["finger_print"].get<std::string>();
                    RCLCPP_INFO(this->get_logger(),
                                "Message for this execution location found: %s",
                                name.c_str());
                    RCLCPP_INFO(this->get_logger(), "Package: %s", package.c_str());
                    RCLCPP_INFO(this->get_logger(), "Messages: %s", messages.c_str());
                    RCLCPP_INFO(this->get_logger(), "Version: %s", version.c_str());
                    RCLCPP_INFO(this->get_logger(), "Location: %s", location.c_str());
                    RCLCPP_INFO(this->get_logger(), "Check persistence of %s", package.c_str());
                    if(m_persistence_handler->check_message(
                           messages, version, location, finger_print))
                    {
                        // Message lib is available try to load it
                        RCLCPP_INFO(
                            this->get_logger(), "Message %s is available", messages.c_str());
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get %s", messages.c_str());
                        ret_val = false;
                    }
                }
            }
            // Iterate through nodes entries in the config file
            nlohmann::json nodes_json = json_object["nodes"];
            for(auto node_json : nodes_json)
            {
                std::string execution_location =
                    node_json["execution_location"].get<std::string>();
                /*Check if the found execution location is this*/
                if(execution_location.compare(m_exec_location) == 0)
                {
                    /*Load all neccesary parameters for persistence management/download*/
                    std::string execution_context =
                        node_json["execution_context"].get<std::string>();
                    std::string name = node_json["name"].get<std::string>();
                    std::string package = node_json["package"].get<std::string>();
                    std::string node = node_json["node"].get<std::string>();
                    std::string version = node_json["version"].get<std::string>();
                    std::string location = node_json["package_location"].get<std::string>();
                    std::string finger_print = node_json["finger_print"].get<std::string>();
                    RCLCPP_INFO(this->get_logger(),
                                "Node for this execution location found: %s",
                                name.c_str());
                    RCLCPP_INFO(this->get_logger(), "Package: %s", package.c_str());
                    RCLCPP_INFO(this->get_logger(), "Node: %s", node.c_str());
                    RCLCPP_INFO(this->get_logger(), "Version: %s", version.c_str());
                    RCLCPP_INFO(this->get_logger(), "Location: %s", location.c_str());
                    RCLCPP_INFO(
                        this->get_logger(), "Exec Context: %s", execution_context.c_str());
                    // Parse parameters and add them to the NodeOptions
                    rclcpp::NodeOptions options;
                    options.allow_undeclared_parameters(true);
                    options.automatically_declare_parameters_from_overrides(true);
                    options.use_intra_process_comms(true);
                    if(node_json.contains("parameters"))
                    {
                        auto parameters_json = node_json["parameters"];

                        for(auto parameter_json : parameters_json)
                        {
                            std::string parameter_name =
                                parameter_json["name"].get<std::string>();
                            std::string param_value = parameter_json["type"].get<std::string>();
                            RCLCPP_INFO(this->get_logger(),
                                        "Parameter found Name: %s, Type %s",
                                        parameter_name.c_str(),
                                        param_value.c_str());

                            if(parameter_json["type"].get<std::string>() == "string")
                            {
                                std::string value = parameter_json["value"].get<std::string>();
                                options.append_parameter_override(parameter_name, value);
                            }
                            else if(parameter_json["type"].get<std::string>() == "int")
                            {
                                int32_t value = parameter_json["value"].get<int32_t>();
                                options.append_parameter_override(parameter_name, value);
                            }
                            /*
                            else if (parameter_json["type"].get<std::string>() == "uint")
                            {
                                uint32_t value = parameter_json["value"].get<uint32_t>();
                                options.append_parameter_override(parameter_name, value);
                            }
                            */
                            else if(parameter_json["type"].get<std::string>() == "float")
                            {
                                double value = parameter_json["value"].get<double>();
                                options.append_parameter_override(parameter_name, value);
                            }
                        }
                    }
                    options.append_parameter_override("routes", msg_routes_json.dump());
                    // check if node is already downloaded if not it is automatically downloaded
                    RCLCPP_INFO(this->get_logger(), "Check pesistence of %s", package.c_str());
                    if(m_persistence_handler->check_package(
                           node, version, location, finger_print))
                    {
                        // Node lib is available try to load it
                        RCLCPP_INFO(this->get_logger(),
                                    "%s is available, start instatntiation",
                                    package.c_str());
                        if(m_exec_handler.add_node_to_context(name,
                                                              execution_context,
                                                              m_config_path + "/lib_install/" +
                                                                  "lib" + node + ".so",
                                                              options))
                        {
                            // Node was successfully instantiated from external lib, add the node to
                            // the lifecycle manager
                            NodeCfg ncf;
                            ncf.m_current_state =
                                lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
                            ncf.m_name = name;
                            ncf.m_node = node;
                            ncf.m_package = package;
                            ncf.m_set_state =
                                lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
                            ncf.m_version = version;

                            m_lifecycle_manager.add_node(ncf);

                            RCLCPP_INFO(
                                this->get_logger(), "Node %s succesfully loaded", name.c_str());
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get %s", package.c_str());
                        ret_val = false;
                    }
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        ret_val = false;
        RCLCPP_ERROR(this->get_logger(), "System Config failed: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "System Config Finished");
    m_system_config_loaded = true;
    return (ret_val);
}

bool ConfigurationHandler::base_config_loaded()
{
    return (m_base_config_loaded);
}
bool ConfigurationHandler::system_config_loaded()
{
    return (m_system_config_loaded);
}
std::string ConfigurationHandler::get_execution_location()
{
    return (m_exec_location);
}
} // namespace config