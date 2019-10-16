#include "executioncontext.hpp"
#include "executionhandler.hpp"
namespace config
{
ExecutionHandler::ExecutionHandler() {}
bool ExecutionHandler::add_node_to_context(std::string node_name, std::string execution_context,
                                           std::string lib_to_load, rclcpp::NodeOptions options)
{
    // Check if execution context already exists
    if (m_execution_contexts.find(execution_context) == m_execution_contexts.end())
    {
        // doesn't exists so create a new one
        RCLCPP_INFO(this->get_logger(), "Created new context %s", execution_context.c_str());
        m_execution_contexts[execution_context] = new ExecutionContext(execution_context);
    }
    // add the node to the execution context
    return (m_execution_contexts[execution_context]->add_node(node_name, lib_to_load, options));
}

bool ExecutionHandler::start_threads()
{
    // Start execution of all execution contexts
    for (auto exec : m_execution_contexts)
    {
        exec.second->start_execution();
    }
}
bool ExecutionHandler::cleanup()
{
    // Shutdown/Clean up all execution contexts
    for (auto exec : m_execution_contexts)
    {
        exec.second->stop_execution();
        delete (exec.second);
    }
    m_execution_contexts.clear();
}

ExecutionHandler::~ExecutionHandler() {}
}  // namespace config