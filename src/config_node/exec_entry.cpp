#include "rclcpp/rclcpp.hpp"

#include "confignode.hpp"
/**
 * @brief Main function of the ConfigNode_exec
 */

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.use_intra_process_comms(true);
    options.append_parameter_override("path", "./config");
    rclcpp::spin(std::make_shared<config::ConfigNode>(options));
    rclcpp::shutdown();
    return 0;
}