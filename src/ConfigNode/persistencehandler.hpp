#pragma once

#include <fstream>
#include <string>
#include <unordered_map>
#include "json/include/nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
namespace config
{
class PersistenceHandler
{
   private:
    // path to working folder
    std::string m_path;
    // Vehicle name/id
    std::string m_vehicle;
    // repository of local available packages
    std::unordered_map<std::string, nlohmann::json> m_repository;
    /**
     * Reset local repository and enforce a new download
     **/
    void reset_repository();
    /**
     * Store repository to json
     **/
    void store_repository();
    // Helper for logger
    inline rclcpp::Logger get_logger()
    {
        return (rclcpp::get_logger("CFG_PER"));
    }

   public:
    /**
     * Constructor
     * @param std::string path path to working directory
     * @param std::string vehicle_name vehicle name/id
     **/
    PersistenceHandler(std::string path, std::string vehicle_name);
    ~PersistenceHandler();
    /**
     * Check if node is available and tries to donwnload it if neccesary
     * @param const std::string& name package/node name
     * @param const std::string& version package/node version
     * @param const std::string& dowsnload_path URL for download
     * @param const std::string& fingerprint MD5 fingerprint of download archive
     **/
    bool check_package(const std::string& name, const std::string& version,
                       const std::string& dowsnload_path, const std::string& fingerprint);
    /**
     * Check if message lib is available and tries to donwnload it if neccesary
     * @param const std::string& name message name
     * @param const std::string& version package/message version
     * @param const std::string& dowsnload_path URL for download
     * @param const std::string& fingerprint MD5 fingerprint of download archive
     **/
    bool check_message(const std::string& name, const std::string& version,
                       const std::string& dowsnload_path, const std::string& fingerprint);
};
}  // namespace config
