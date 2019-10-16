#include "rclcpp/rclcpp.hpp"
#include "rtcmnode.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // options
    rclcpp::executors::SingleThreadedExecutor exe;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.use_intra_process_comms(true);

    options.append_parameter_override("ntrip_host", "157.97.109.112");
    options.append_parameter_override("ntrip_port", 2101);
    options.append_parameter_override("ntrip_mountpoint", "VRS_3_2G_BE");
    options.append_parameter_override("ntrip_user", "saposbln287-2");
    options.append_parameter_override("ntrip_password", "gb32wm");
    options.append_parameter_override("node_name", "rtcm");
    std::shared_ptr<rtcm::RtcmNode> node = std::make_shared<rtcm::RtcmNode>(options);

    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}