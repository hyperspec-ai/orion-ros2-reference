#pragma once
#include <archive.h>
#include <archive_entry.h>
#include <curl/curl.h>
#include <stdio.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
namespace config {
class PackageDownloader
{
  private:
    /**
     * unpacks the defined package of a node (exactly one file, the .so lib file of the node)
     * @param archive_path path to the archive (tar.gz)
     * @param target_path Path were the file should be deflated
     * @param package_name Name of the Node/Implemented Node class
     *
     * @return true if operation was succesfull
     **/
    static bool unpack_package(const std::string& archive_path,
                               const std::string& target_path,
                               const std::string& package_name);

    /**
     * unpacks the defined package of a message (exactly three files, the .so lib file ending with
     *cpp)
     * @param archive_path path to the archive (tar.gz)
     * @param target_path Path were the file should be deflated
     * @param message_name Name of the Node/Implemented Node class
     *
     * @return true if operation was succesfull
     **/
    static bool unpack_message(const std::string& archive_path,
                               const std::string& target_path,
                               const std::string& message_name);

    /**
     * Helper function to store data from arhcive to disk
     *
     * @param *ar input archive
     * @param *aw output archive/disk
     *
     * @return error code
     **/
    static int copy_data(struct archive* ar, struct archive* aw);
    /**
     * Helper function to store download to file
     * @param ptr Pointer to the array of elements to be written, converted to a const void*.
     * @param size Size in bytes of each element to be written
     * @param nmemb Number of elements, each one with a size of size bytes.
     * @param stream Pointer to file stream
     **/
    static size_t write_data(void* ptr, size_t size, size_t nmemb, FILE* stream);

    //! Helper function for logging
    static inline rclcpp::Logger get_logger() { return (rclcpp::get_logger("CFG_DOW")); }

  public:
    /**
     * Downloads and deflates package to the given location
     *
     * @param url_to_file URL for file to download
     * @param md5_fingerprint MD5 Fingerprint of the archive (check not yet implemented)
     * @param target_path Path were the file should be deflated
     * @param package_name Name of the Node/Implemented Node class
     *
     * @return true in case file was succesfully downloaded and deflated
     **/
    static bool download_package(const std::string& url_to_file,
                                 const std::string& md5_fingerprint,
                                 const std::string& target_path,
                                 const std::string& package_name);
    /**
     * Downloads and deflates package to the given location
     *
     * @param url_to_file URL for file to download
     * @param md5_fingerprint MD5 Fingerprint of the archive (check not yet implemented)
     * @param target_path Path were the file should be deflated
     * @param package_name Name of the Node/Implemented Node class
     *
     * @return true in case file was succesfully downloaded and deflated
     **/
    static bool download_message(const std::string& url_to_file,
                                 const std::string& md5_fingerprint,
                                 const std::string& target_path,
                                 const std::string& package_name);
};
} // namespace config