#include "lifecyclemanager.hpp"
using namespace std::chrono_literals;
namespace config
{
LifecycleManager::LifecycleManager(rclcpp::Node* node)
{
    m_config_node = node;
}

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do
    {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0))
        {
            break;
        }
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

bool LifecycleManager::change_state(std::uint8_t transition, std::chrono::seconds time_out,
                                    NodeCfg& node)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!node.m_client_change_state->wait_for_service(time_out))
    {
        RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                     node.m_client_change_state->get_service_name());
        return false;
    }
    // We send the request with the transition we want to invoke.
    auto future_result = node.m_client_change_state->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s",
                     node.m_name.c_str());
        return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success)
    {
        RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.",
                    static_cast<int>(transition));
        return true;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Failed to trigger transition %u",
                    static_cast<unsigned int>(transition));
        return false;
    }
}
bool LifecycleManager::set_all_state(std::uint8_t transition, std::chrono::seconds time_out)
{
    bool ret_val = true;
    for (NodeCfg node : m_configured_nodes)
    {
        if (!change_state(transition, time_out, node))
        {
            ret_val = false;
        }
    }
    return (ret_val);
}
bool LifecycleManager::set_all_configured()
{
    return (set_all_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 5s));
}
bool LifecycleManager::set_all_active()
{
    return (set_all_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 20s));
}
bool LifecycleManager::set_all_deactive()
{
    return (set_all_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 5s));
}

bool LifecycleManager::set_all_shutdown()
{
    m_configured_nodes.clear();
    return (set_all_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, 5s));
}

void LifecycleManager::add_node(NodeCfg nfc)
{
    std::string cs_topic = nfc.m_name + "/change_state";
    std::string gs_topic = nfc.m_name + "/get_state";

    nfc.m_client_change_state =
        m_config_node->create_client<lifecycle_msgs::srv::ChangeState>(cs_topic.c_str());

    nfc.m_client_get_state =
        m_config_node->create_client<lifecycle_msgs::srv::GetState>(gs_topic.c_str());

    RCLCPP_INFO(get_logger(), "Creating for Node %s lifecycle client, with topic %s and %s.",
                nfc.m_name.c_str(), cs_topic.c_str(), gs_topic.c_str());
    m_configured_nodes.push_back(nfc);
}

bool LifecycleManager::check_state_all_nodes()
{
    for (NodeCfg node : m_configured_nodes)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        std::chrono::seconds time_out = 10s;
        if (!node.m_client_get_state->wait_for_service(time_out))
        {
            RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                         node.m_client_get_state->get_service_name());
        }

        // We send the service request for asking the current
        // state of the lc_talker node.
        auto future_result = node.m_client_get_state->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s",
                         node.m_name.c_str());
            continue;
        }

        // We have an succesful answer. So let's print the current state.
        if (future_result.get())
        {
            RCLCPP_INFO(get_logger(), "Node %s has current state %s.", node.m_name.c_str(),
                        future_result.get()->current_state.label.c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s",
                         node.m_name.c_str());
        }
    }
}
LifecycleManager::~LifecycleManager() {}

}  // namespace config