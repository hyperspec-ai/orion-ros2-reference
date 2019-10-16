#include "rclcpp/rclcpp.hpp"
#include "ubxgpsnode.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // options
    rclcpp::executors::SingleThreadedExecutor exe;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.use_intra_process_comms(true);
    options.append_parameter_override("serial_port", "/dev/ttyUB0");
    options.append_parameter_override("sampling_time", 60);
    options.append_parameter_override("rtcm_topic", "/rtcm/rtcm_data");
    options.append_parameter_override("node_name", "ubx_gnss");
    std::shared_ptr<ubx_node::UbxGpsNode> node = std::make_shared<ubx_node::UbxGpsNode>(options);
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
