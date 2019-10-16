#include "executioncontext.hpp"
namespace config
{
ExecutionContext::ExecutionContext(std::string name)
{
    m_name = name;
    m_running = false;
}
ExecutionContext::~ExecutionContext() {}
bool ExecutionContext::add_node(std::string name, std::string lib_path,
                                rclcpp::NodeOptions& options)
{
    bool ret_val = true;
    try
    {
        auto loader = new class_loader::ClassLoader(lib_path);
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        RCLCPP_INFO(this->get_logger(), "Start interating class loaders for lib path %s",
                    lib_path.c_str());
        for (auto clazz : classes)
        {
            /*Prevent loading Config node class*/
            if (clazz.compare("rclcpp_components::NodeFactoryTemplate<config::ConfigNode>") != 0)
            {
                RCLCPP_INFO(this->get_logger(), "Load node factory");
                auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
                RCLCPP_INFO(this->get_logger(), "Set node name to %s", name.c_str());
                options.append_parameter_override("node_name", name);
                RCLCPP_INFO(this->get_logger(), "Instantiate class %s", clazz.c_str());
                auto wrapper = node_factory->create_node_instance(options);
                RCLCPP_INFO(this->get_logger(), "Get Base Node Interface");
                auto node = wrapper.get_node_base_interface();
                m_node_wrappers.push_back(wrapper);
                RCLCPP_INFO(this->get_logger(), "Add node to executor");
                m_executor.add_node(node);
                RCLCPP_INFO(this->get_logger(), "Added to %s node %s for execution", m_name.c_str(),
                            name.c_str());
            }
        }
        m_loaders.push_back(loader);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Load of node failed %s with reason %s", name.c_str(),
                     e.what());
        ret_val = false;
    }
    return (ret_val);
}

void ExecutionContext::start_execution()
{
    m_running = true;
    RCLCPP_INFO(this->get_logger(), "Worker Thread for %s started", m_name.c_str());
    m_worker_thread = new std::thread([this] { worker_thread(); });
}

void ExecutionContext::stop_execution()
{
    /*Set flag to stop thtrads and wait until finished*/
    m_running = false;
    m_worker_thread->join();
    /*Deatroy dynamically allocated objects*/
    delete (m_worker_thread);
    m_worker_thread = nullptr;

    /*remove all instances of loaders*/
    m_loaders.clear();
    for (auto wrapper : m_node_wrappers)
    {
        m_executor.remove_node(wrapper.get_node_base_interface());
    }
    /*Ensure that all timers are killed*/
    m_executor.cancel();

    m_node_wrappers.clear();
}

void ExecutionContext::worker_thread()
{
    RCLCPP_INFO(this->get_logger(), "Worker Thread for execution context started (%s)",
                m_name.c_str());
    while (m_running)
    {
        m_executor.spin_some();
    }
    RCLCPP_INFO(this->get_logger(), "Worker Thread for execution context stopped (%s)",
                m_name.c_str());
}
}  // namespace config