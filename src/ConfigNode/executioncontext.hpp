#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <unordered_map>
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
namespace config
{
/*!
 * Execution Context reflects a ROS2 Singlethreaded executor enabling intraprocess communication
 * between the nodes inside a execution contex
 **/
class ExecutionContext
{
   private:
    /**
     * Spinning thread for ROS2 single thread executor
     * @return true in case unload was succesfull
     */
    void worker_thread();
    // Helper function for logging
    inline rclcpp::Logger get_logger()
    {
        return (rclcpp::get_logger("CFG_EXC_" + m_name));
    }
    // ROS2 Single thread executor
    rclcpp::executors::SingleThreadedExecutor m_executor;
    // Reference to thread object for worker thread
    std::thread* m_worker_thread;
    // Name of the execution context
    std::string m_name;
    // Thread controll variable for shutdown
    std::atomic<bool> m_running;
    // Collection of executed nodes
    std::vector<rclcpp_components::NodeInstanceWrapper> m_node_wrappers;
    // Collection of class loader per node
    std::vector<class_loader::ClassLoader*> m_loaders;

   public:
    /**
     * Constructor
     *
     * @param std::string name Execution context name
     */
    ExecutionContext(std::string name);
    /**
     * Adds a node to the execution context defined by the li in the lib_path
     *
     * @param std::string name Node Name
     * @param std::string lib_path path to library, including filename (.so)
     * @param rclcpp::NodeOptions& options Node options generated from system config file parameter
     * section
     *
     * @return true if instantiation from lib file was sucessfull
     */
    bool add_node(std::string name, std::string lib_path, rclcpp::NodeOptions& options);
    /**
     * Starts the worker thread
     **/
    void start_execution();
    /**
     * Stops the worker thread and cleans up, all nodes are unegeisted after that
     **/
    void stop_execution();
    /**
     * Destructor
     **/
    ~ExecutionContext();
};
}  // namespace config