#include "rclcpp/rclcpp.hpp"

#include "confignode.hpp"
/**
 * @brief Main function of the ConfigNode_exec
 */

int main(int argc, char* argv[])
{
    //TODO: make explicit parsing for path as this can also be used for rclcpp parameters
    if(argc == 2)
    {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);
        options.use_intra_process_comms(true);
        options.append_parameter_override("path", argv[1]);
        rclcpp::spin(std::make_shared<config::ConfigNode>(options));
        rclcpp::shutdown();
    }
    else
    {
        printf("Wrong number of parameters");
    }
    return 0;
}