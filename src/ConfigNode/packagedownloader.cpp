#include <iostream>
#include "packagedownloader.hpp"
namespace config {
std::string get_filename_from_url(const std::string& s)
{
    char sep = '/';

    size_t i = s.rfind(sep, s.length());
    if(i != std::string::npos)
    {
        return (s.substr(i + 1, s.length() - i));
    }

    return ("");
}

bool PackageDownloader::download_package(const std::string& url_to_file,
                                         const std::string& md5_fingerprint,
                                         const std::string& target_path,
                                         const std::string& package_name)
{
    CURL* curl;
    FILE* fp;
    CURLcode res;
    bool ret_val = false;
    const char* url = url_to_file.c_str();
    //TODO : Add md5 check
    std::ignore = md5_fingerprint;
    RCLCPP_INFO(get_logger(), "Download Package from: %s", url_to_file.c_str());
    std::string file_name = get_filename_from_url(url_to_file);
    std::string target_file = "";
    target_file = "/tmp/" + file_name;
    const char* outfilename = target_file.c_str();
    curl = curl_easy_init();
    if(curl)
    {
        fp = fopen(outfilename, "wb");
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        res = curl_easy_perform(curl);
        /* always cleanup */
        curl_easy_cleanup(curl);
        RCLCPP_INFO(get_logger(),
                    "Download Package finished: %s with result %d ",
                    url_to_file.c_str(),
                    res);
        fclose(fp);
        ret_val = unpack_package(target_file, target_path, package_name);
    }
    return (ret_val);
}

bool PackageDownloader::download_message(const std::string& url_to_file,
                                         const std::string& md5_fingerprint,
                                         const std::string& target_path,
                                         const std::string& package_name)
{
    CURL* curl;
    FILE* fp;
    CURLcode res;
    bool ret_val = false;
    const char* url = url_to_file.c_str();
    //TODO : Add md5 check
    std::ignore = md5_fingerprint;
    RCLCPP_INFO(get_logger(), "Download Package from: %s", url_to_file.c_str());
    std::string file_name = get_filename_from_url(url_to_file);
    std::string target_file = "";
    target_file = "/tmp/" + file_name;
    const char* outfilename = target_file.c_str();
    curl = curl_easy_init();
    if(curl)
    {
        fp = fopen(outfilename, "wb");
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        res = curl_easy_perform(curl);
        /* always cleanup */
        curl_easy_cleanup(curl);
        RCLCPP_INFO(get_logger(),
                    "Download Package finished: %s with result %d ",
                    url_to_file.c_str(),
                    res);
        fclose(fp);
        ret_val = unpack_message(target_file, target_path, package_name);
    }
    return (ret_val);
}

size_t PackageDownloader::write_data(void* ptr, size_t size, size_t nmemb, FILE* stream)
{
    size_t written = fwrite(ptr, size, nmemb, stream);
    RCLCPP_INFO(get_logger(), "Write chunk of download");
    return written;
}

bool PackageDownloader::unpack_package(const std::string& archive_path,
                                       const std::string& target_path,
                                       const std::string& package_name)
{
    bool ret_val = true;
    struct archive* a;
    struct archive* ext;
    struct archive_entry* entry;
    RCLCPP_INFO(get_logger(), "Open Package archive from: %s", archive_path.c_str());

    std::string lib_name = "lib" + package_name + ".so";
    int flags;
    int r;
    // Set flags for deflated files
    flags = ARCHIVE_EXTRACT_TIME;
    flags |= ARCHIVE_EXTRACT_PERM;
    flags |= ARCHIVE_EXTRACT_ACL;
    flags |= ARCHIVE_EXTRACT_FFLAGS;
    RCLCPP_INFO(get_logger(), "Seaching for: %s", lib_name.c_str());
    // create archive reader
    a = archive_read_new();
    archive_read_support_format_all(a);
    archive_read_support_filter_all(a);
    // Create writer to write deflated files to disk
    ext = archive_write_disk_new();
    archive_write_disk_set_options(ext, flags);
    archive_write_disk_set_standard_lookup(ext);
    // Open archive
    if((r = archive_read_open_filename(a, archive_path.c_str(), 10240)))
    {
        ret_val = false;
        RCLCPP_ERROR(get_logger(), "Could not open archive %s", archive_error_string(ext));
    }
    else
    {
        for(;;)
        {
            // loop through all entries in the archive
            r = archive_read_next_header(a, &entry);
            // Check on end
            if(r == ARCHIVE_EOF)
            {
                break;
            }
            // check on errors
            else if(r < ARCHIVE_OK)
            {
                RCLCPP_ERROR(
                    get_logger(), "Read data to target failed %s", archive_error_string(ext));
                ret_val = false;
                break;
            }

            std::string entry_path(archive_entry_pathname(entry));
            RCLCPP_INFO(get_logger(), "Found: %s", entry_path.c_str());
            // If the found file/entry has the correct/expected file name deflate
            if(entry_path.compare(lib_name) == 0)
            {
                RCLCPP_INFO(get_logger(), "Unpack Node Shared Lib: %s", lib_name.c_str());
                std::string target_file = target_path + "/" + lib_name;
                RCLCPP_INFO(get_logger(), "Unpack to: %s", target_file.c_str());
                archive_entry_set_pathname(entry, target_file.c_str());
                r = archive_write_header(ext, entry);
                // Check success
                if(r < ARCHIVE_OK)
                    fprintf(stderr, "%s\n", archive_error_string(ext));
                else if(archive_entry_size(entry) > 0)
                {
                    r = copy_data(a, ext);
                    if(r < ARCHIVE_OK)
                    {
                        RCLCPP_ERROR(get_logger(),
                                     "Writing data to target failed %s",
                                     archive_error_string(ext));
                        ret_val = false;
                        break;
                    }
                }
                r = archive_write_finish_entry(ext);
                if(r < ARCHIVE_OK)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Writing final data to target failed %s",
                                 archive_error_string(ext));
                    ret_val = false;
                    break;
                }

                break;
            }
        }
    }
    archive_read_close(a);
    archive_read_free(a);
    archive_write_close(ext);
    archive_write_free(ext);
    return (ret_val);
}

bool PackageDownloader::unpack_message(const std::string& archive_path,
                                       const std::string& target_path,
                                       const std::string& message_name)
{
    bool ret_val = true;
    struct archive* a;
    struct archive* ext;
    struct archive_entry* entry;
    RCLCPP_INFO(get_logger(), "Open Message archive from: %s", archive_path.c_str());

    std::string lib_name_1 = "lib" + message_name + "__rosidl_typesupport_cpp.so";
    std::string lib_name_2 = "lib" + message_name + "__rosidl_typesupport_fastrtps_cpp.so";
    std::string lib_name_3 = "lib" + message_name + "__rosidl_typesupport_introspection_cpp.so";
    int flags;
    int r;
    // Set flags for deflated files
    flags = ARCHIVE_EXTRACT_TIME;
    flags |= ARCHIVE_EXTRACT_PERM;
    flags |= ARCHIVE_EXTRACT_ACL;
    flags |= ARCHIVE_EXTRACT_FFLAGS;
    // create archive reader
    a = archive_read_new();
    archive_read_support_format_all(a);
    archive_read_support_filter_all(a);
    // Create writer to write deflated files to disk
    ext = archive_write_disk_new();
    archive_write_disk_set_options(ext, flags);
    archive_write_disk_set_standard_lookup(ext);
    // Open archive
    if((r = archive_read_open_filename(a, archive_path.c_str(), 10240)))
    {
        ret_val = false;
        RCLCPP_ERROR(get_logger(), "Could not open archive %s", archive_error_string(ext));
    }
    else
    {
        for(;;)
        {
            // loop through all entries in the archive
            r = archive_read_next_header(a, &entry);
            // Check on end
            if(r == ARCHIVE_EOF)
            {
                break;
            }
            // check on errors
            else if(r < ARCHIVE_OK)
            {
                RCLCPP_ERROR(
                    get_logger(), "Read data to target failed %s", archive_error_string(ext));
                ret_val = false;
                break;
            }

            std::string entry_path(archive_entry_pathname(entry));
            RCLCPP_INFO(get_logger(), "Found: %s", entry_path.c_str());
            // If the found file/entry has the correct/expected file name deflate
            if((entry_path.compare(lib_name_1) == 0) || (entry_path.compare(lib_name_2) == 0) ||
               (entry_path.compare(lib_name_3) == 0))
            {
                RCLCPP_INFO(get_logger(), "Unpack Node Shared Lib: %s", entry_path.c_str());
                std::string target_file = target_path + "/" + entry_path;
                RCLCPP_INFO(get_logger(), "Unpack to: %s", target_file.c_str());
                archive_entry_set_pathname(entry, target_file.c_str());
                r = archive_write_header(ext, entry);
                // Check success
                if(r < ARCHIVE_OK)
                    fprintf(stderr, "%s\n", archive_error_string(ext));
                else if(archive_entry_size(entry) > 0)
                {
                    r = copy_data(a, ext);
                    if(r < ARCHIVE_OK)
                    {
                        RCLCPP_ERROR(get_logger(),
                                     "Writing data to target failed %s",
                                     archive_error_string(ext));
                        ret_val = false;
                        break;
                    }
                }
                r = archive_write_finish_entry(ext);
                if(r < ARCHIVE_OK)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Writing final data to target failed %s",
                                 archive_error_string(ext));
                    ret_val = false;
                    break;
                }
            }
        }
    }
    archive_read_close(a);
    archive_read_free(a);
    archive_write_close(ext);
    archive_write_free(ext);
    return (ret_val);
}

int PackageDownloader::copy_data(struct archive* ar, struct archive* aw)
{
    int r;
    const void* buff;
    size_t size;
    la_int64_t offset;

    for(;;)
    {
        r = archive_read_data_block(ar, &buff, &size, &offset);
        if(r == ARCHIVE_EOF) return (ARCHIVE_OK);
        if(r < ARCHIVE_OK) return (r);
        r = archive_write_data_block(aw, buff, size, offset);
        if(r < ARCHIVE_OK)
        {
            break;
        }
    }
    return (r);
}
} // namespace config