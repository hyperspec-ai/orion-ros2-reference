#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <unordered_map>
#include "class_loader/class_loader.hpp"
#include "executioncontext.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

namespace config
{
class ExecutionHandler
{
   private:
    // Collection of all execution contexts
    std::unordered_map<std::string, ExecutionContext *> m_execution_contexts;
    // Helper for logging
    inline rclcpp::Logger get_logger()
    {
        return (rclcpp::get_logger("CFG_EXH"));
    }

   public:
    /**
     * Constructor
     **/
    ExecutionHandler();
    /**
     * Adds a node to the given context
     *
     * @param std::string node_name Name of the node
     * @param std::string execution_context name of the excution context the node should be executed
     * @param std::string lib_to_load path including filname to lib (.so) of node
     * @param rclcpp::NodeOptions options Node options loaded from system configuration parameter
     * section
     *
     * @returm true if insatntiation/adding was succesfull
     **/
    bool add_node_to_context(std::string node_name, std::string execution_context,
                             std::string lib_to_load, rclcpp::NodeOptions options);
    /**
     * Start all execution context threads
     **/
    bool start_threads();
    /**
     * Stop and clean up all execution context threads
     **/
    bool cleanup();

    /**
     * Desctuctor
     **/
    ~ExecutionHandler();
};
}  // namespace config