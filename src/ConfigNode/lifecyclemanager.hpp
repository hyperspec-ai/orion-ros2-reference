#pragma once
#include <chrono>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nodecfg.hpp"
#include "rclcpp/rclcpp.hpp"

namespace config {
class LifecycleManager
{
  private:
    //! Configured nodes with lifecycle interface
    std::vector<NodeCfg> m_configured_nodes;
    //! Reference to ConfigNode, needed to create life cylce client services
    rclcpp::Node* m_config_node;
    //! Helper for logging
    inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("CFG_LCM")); }

  public:
    /**
     * Constructor
     * @param node Reference to ConfigNode
     **/
    LifecycleManager(rclcpp::Node* node);
    /**
     * Change the state of the node
     *
     * @param transition tranisition accroding to lifecycle_msgs::msg::Transition
     * @param time_out timeout in seconds
     * @param node node config to change the state for
     *
     * @return true succesfull state change
     **/
    bool change_state(std::uint8_t transition, std::chrono::seconds time_out, NodeCfg& node);
    /**
     * Change the state of all nodes registered
     *
     * @param transition tranisition accroding to lifecycle_msgs::msg::Transition
     * @param time_out timeout in seconds
     *
     * @return true succesfull state change
     **/
    bool set_all_state(std::uint8_t transition, std::chrono::seconds time_out);
    /**
     * Checks the state of all nodes if they have the expected state (WIP)
     *
     * @return true if all nodes are in the expected state
     **/
    bool check_state_all_nodes();
    /**
     * Set all nodes in the state configured
     *
     * @return state transition sucesfull
     **/
    bool set_all_configured();
    /**
     * Set all nodes in the state active
     *
     * @return state transition sucesfull
     **/
    bool set_all_active();
    /**
     * Set all nodes in the state deactivated
     *
     * @return state transition sucesfull
     **/
    bool set_all_deactive();
    /**
     * Set all nodes in the state cleanup
     *
     * @return state transition sucesfull
     **/
    bool set_all_shutdown();
    /**
     * Adds a node to the lifecycl manager
     *
     * @param ncf Node Config to register
     **/
    void add_node(NodeCfg ncf);
    /**
     * @brief Destroy the Lifecycle Manager object
     * 
     */
    ~LifecycleManager();
};
} // namespace config
