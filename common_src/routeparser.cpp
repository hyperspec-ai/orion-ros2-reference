#include "routeparser.hpp"

RouteParser::RouteParser(const std::string& routes_json, const std::string& node_name)
{
    m_routes = nlohmann::json::parse(routes_json);
    m_node_name = node_name;
}
std::string RouteParser::get_topic_by_endpoint(std::string endpoint)
{
    std::string ret_val = "/not/found";
    try
    {
        /* code */

        for(auto route_entry : m_routes)
        {
            auto dests = route_entry["dst"];
            for(auto dest_info : dests)
            {
                //                RCLCPP_INFO(this->get_logger(),
                //                            "Found destination endpoint: %s",
                //                            dest_info.dump().c_str());
                if((dest_info["node"].get<std::string>().compare(m_node_name) == 0) &&
                   (dest_info["endpoint"].get<std::string>().compare(endpoint) == 0))
                {
                    ret_val = "/" + route_entry["src"]["node"].get<std::string>() + "/" +
                        route_entry["src"]["endpoint"].get<std::string>();
                }
            }
        }
    }
    catch(const std::exception& e)
    {
    }
    return (ret_val);
}
