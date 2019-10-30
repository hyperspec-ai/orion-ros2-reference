#pragma once

#include <fstream>
#include <string>
#include <unordered_map>
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
namespace config {
class PersistenceHandler
{
  private:
    //! path to working folder
    std::string m_path;
    //! Vehicle name/id
    std::string m_vehicle;
    //! repository of local available packages
    std::unordered_map<std::string, nlohmann::json> m_repository;
    /**
     * Reset local repository and enforce a new download
     **/
    void reset_repository();
    /**
     * Store repository to json
     **/
    void store_repository();
    //! Helper for logger
    inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("CFG_PER")); }

  public:
    /**
     * Constructor
     * @param path path to working directory
     * @param vehicle_name vehicle name/id
     **/
    PersistenceHandler(std::string path, std::string vehicle_name);
    ~PersistenceHandler();
    /**
     * Check if node is available and tries to donwnload it if neccesary
     * @param name package/node name
     * @param version package/node version
     * @param dowsnload_path URL for download
     * @param fingerprint MD5 fingerprint of download archive
     **/
    bool check_package(const std::string& name,
                       const std::string& version,
                       const std::string& dowsnload_path,
                       const std::string& fingerprint);
    /**
     * Check if message lib is available and tries to donwnload it if neccesary
     * @param name message name
     * @param version package/message version
     * @param dowsnload_path URL for download
     * @param fingerprint MD5 fingerprint of download archive
     **/
    bool check_message(const std::string& name,
                       const std::string& version,
                       const std::string& dowsnload_path,
                       const std::string& fingerprint);
};
} // namespace config
