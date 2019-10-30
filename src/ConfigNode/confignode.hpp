#pragma once
#include <atomic>
#include <thread>
#include "configurationhandler.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nodecfg.hpp"
#include "rclcpp/rclcpp.hpp"

namespace config {
/**
 * ROS2 Node careing about configuration/instantiation/update
 **/

class ConfigNode : public rclcpp::Node
{
  public:
    /**
     * Constructor
     *
     * @param options Node options the config node should be initilaized
     */
    ConfigNode(const rclcpp::NodeOptions& options);
    /**
     * Destructor
     */
    virtual ~ConfigNode();

  private:
    //!Indicates in case of false shutdown of all threads
    std::atomic<bool> m_running;
    //!Thread object for worker thread
    std::thread* m_executor;
    //!Config and working path for the system
    rclcpp::Parameter m_path;
    //!Pointer to the configuration handler instance
    ConfigurationHandler* m_config_handler;
    /**
     * Worker thread calling Configuration handler and caring about life cycle management
     **/
    void worker_thread();
    /**
     * Life cycle: Function setting all nodes from uncronfigured-->configured-->active
     * @return true if all nodes reaches active states
     **/
    bool activate_all_nodes();
    /**
     * Life cycle: Checks all node states
     * @return true if all states can be requested and fit to the expected state
     **/
    bool check_state_all_nodes();
    /**
     * Life Cycle: helper function to set a state to a specific value
     *
     * @param transition requested transition according to transition states from ROS2
     * @param time_out Timeout time for requested transition
     * @param node Node Cfg object holding neccesary information to check/set states
     */
    bool change_state(std::uint8_t transition, std::chrono::seconds time_out, NodeCfg& node);
};
} // namespace config
