#include <iostream>
#include "packagedownloader.hpp"
#include "persistencehandler.hpp"
namespace config
{
PersistenceHandler::PersistenceHandler(std::string path, std::string vehicle_name)
{
    m_path = path;
    m_vehicle = vehicle_name;
    try
    {
        // Open repository jso
        std::string repository_filename = m_path + "/repository/repository.json";
        RCLCPP_INFO(this->get_logger(), "Load repository from %s", repository_filename.c_str());
        std::ifstream json_file(repository_filename);
        nlohmann::json json_object;
        json_file >> json_object;
        // Check if repository has the correct vehicle entry and is not coppied from somewhere else
        if (json_object["vehicle"].get<std::string>() == vehicle_name)
        {
            // Iterate through package enries
            nlohmann::json json_packages = json_object["packages"];
            for (auto json_package : json_packages)
            {
                // load each package entry into a unorderd map for faster access
                std::string package_name = json_package["package"];
                RCLCPP_INFO(this->get_logger(), "Package Found %s in Version %s",
                            package_name.c_str(),
                            json_package["version"].get<std::string>().c_str());
                m_repository[package_name] = json_package;
            }
        }
        else
        {
            // Something went wrong delete content of repository.json and also lib files
            reset_repository();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        reset_repository();
    }
}

PersistenceHandler::~PersistenceHandler() {}

void PersistenceHandler::reset_repository()
{
    m_repository.clear();
    /*ToDo: Delet all ros2 node/message/dependecie files*/
}

bool PersistenceHandler::check_package(const std::string& name, const std::string& version,
                                       const std::string& dowsnload_path,
                                       const std::string& fingerprint)
{
    bool download = true;
    bool success = true;
    // Chech if package is already in list
    if (m_repository.find(name) != m_repository.end())
    {
        // Check for correct version
        if (m_repository[name]["version"] == version)
        {
            // correct version already installed
            download = false;
            RCLCPP_INFO(this->get_logger(), "Package %s in repository found", name.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Package %s in repository found but in the wrong version", name.c_str());
        }
    }
    if (download)
    {
        // Version miss match or generally not downloaded
        RCLCPP_INFO(this->get_logger(), "Try to download correct version from: %s",
                    dowsnload_path.c_str());
        // Download and deflate package
        if (!PackageDownloader::download_package(dowsnload_path, fingerprint,
                                                 m_path + "/lib_install/", name))
        {
            RCLCPP_ERROR(this->get_logger(), "Download failed: %s", dowsnload_path.c_str());
            success = false;
        }
        else
        {
            m_repository[name]["version"] = version;
            m_repository[name]["package"] = name;
            m_repository[name]["type"] = "node";
            store_repository();
        }
    }
    return (success);
}
bool PersistenceHandler::check_message(const std::string& name, const std::string& version,
                                       const std::string& dowsnload_path,
                                       const std::string& fingerprint)
{
    bool download = true;
    bool success = true;
    // Chech if package is already in list
    if (m_repository.find(name) != m_repository.end())
    {
        // Check for correct version
        if (m_repository[name]["version"] == version)
        {
            // correct version already installed
            download = false;
            RCLCPP_INFO(this->get_logger(), "Package %s in repository found", name.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Package %s in repository found but in the wrong version", name.c_str());
        }
    }
    if (download)
    {
        // Version miss match or generally not downloaded
        RCLCPP_INFO(this->get_logger(), "Try to download correct version from: %s",
                    dowsnload_path.c_str());
        // Download and deflate package
        if (!PackageDownloader::download_message(dowsnload_path, fingerprint,
                                                 m_path + "/lib_message/", name))
        {
            RCLCPP_ERROR(this->get_logger(), "Download failed: %s", dowsnload_path.c_str());
            success = false;
        }
        else
        {
            m_repository[name]["version"] = version;
            m_repository[name]["package"] = name;
            m_repository[name]["type"] = "node";
            store_repository();
        }
    }
    return (success);
}
void PersistenceHandler::store_repository()
{
    // Build up json structure
    nlohmann::json target_json;
    // Adding vehiclen name/id entry
    target_json["vehicle"] = m_vehicle;
    // Add packages in the
    std::vector<nlohmann::json> pack;
    for (auto entry : m_repository)
    {
        pack.push_back(entry.second);
    }
    target_json["packages"] = pack;
    // Write to file
    std::string repository_filename = m_path + "/repository/repository.json";
    std::ofstream o(repository_filename);
    o << target_json << std::endl;
}
}  // namespace config