/***
 * NTRIP Client
 * */
#pragma once
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
#include "base64/base64.h"
#include "ntripcallback.hpp"
#include "ntripclient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket.hpp"
namespace ntrip
{
NtripClient::NtripClient(const std::string& host, uint16_t port, const std::string& mount_point,
                         NtripCallback* callback, const std::string& user,
                         const std::string& password)
    : m_host(host), m_port(port), m_mount_point(mount_point), m_callback(callback)
{
    /**
     * Build base 64 encoded auth string
     **/
    std::string auth = user + ":" + password;
    m_auth = base64_encode((uint8_t*)auth.c_str(), auth.size());
    m_running = true;
    RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Starting NTRIP Client thread");
    m_thread = new std::thread([this] { worker_thread(); });
}

void NtripClient::set_gga(std::string gga)
{
    std::lock_guard<std::mutex> lock(m_gga_mutex);
    m_gga = gga;
}
void NtripClient::shutdown()
{
    m_running = false;
    m_thread->join();
}

void NtripClient::worker_thread()
{
    while (m_running)
    {
        RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Starting ntrip client connection");
        addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* info;
        /**
         * Do address resolution
         **/
        if (getaddrinfo(m_host.c_str(), std::to_string(m_port).c_str(), &hints, &info) != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Address Resolution failed %s",
                        m_host.c_str());
            sleep(5);
        }
        else
        {
            /**
             * Create Socket and try to connects
             **/
            Socket socket(InternetProtocol::V4);
            struct timeval timeout;
            timeout.tv_sec = 10;
            timeout.tv_usec = 0;
            if (::setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Ser RCV Timeot failed");
            }
            if (::setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Ser SND Timeot failed");
            }
            RCLCPP_INFO(rclcpp::get_logger("NTRIP"),
                        "Adress resolution succesfull connect to server");
            if (::connect(socket, info->ai_addr, info->ai_addrlen) < 0)
            {
                freeaddrinfo(info);
                RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Connect failed %s:%d", m_host.c_str(),
                            m_port);
                sleep(5);
            }
            else
            {
                /**
                 * Prepare headers
                 **/
                std::vector<std::string> headers = {
                    "Ntrip-Version: Ntrip/2.0"
                    "User-Agent: rtcm_ros",
                    "Connection: close", "Authorization: Basic " + m_auth};
                /**
                 * Prepare Request string
                 **/
                std::string request_data = "GET /" + m_mount_point + " HTTP/1.1\r\n";

                for (const std::string& header : headers)
                {
                    request_data += header + "\r\n";
                }
                request_data += "Host: " + m_host + "\r\n";
                request_data += "Content-Length: 0\r\n";
                request_data += "\r\n";
                /**
                 * Write request to Socket
                 **/
                write_to_socket(socket, request_data);
                recv_success_t recv_state = RECV_OK;
                Response resp = read_from_socket(socket, &recv_state, RECV_START);
                /**
                 * Check if connection/authetication was succesfull
                 */
                if (recv_state == RECV_OK)
                {
                    if (resp.status == Response::STATUS_OK)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Succesfully connected to server");
                        while (m_running)
                        {
                            /*Send ggs NMEA Sentence with current position*/
                            {
                                std::lock_guard<std::mutex> lock(m_gga_mutex);
                                write_to_socket(socket, m_gga);
                            }
                            /*Read RTCM Sentences*/
                            Response resp = read_from_socket(socket, &recv_state, RECV_CONT);
                            if (recv_state == RECV_OK)
                            {
                                if (resp.body.size() != 0)
                                {
                                    /*Deliver data via callback*/
                                    m_callback->data_received(resp.body);
                                }
                            }
                            else
                            {
                                RCLCPP_INFO(rclcpp::get_logger("NTRIP"),
                                            "Connection dropped %s:%d with Code %d", m_host.c_str(),
                                            m_port, resp.status);
                                break;
                                /*Try to reconnect*/
                            }
                        }
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("NTRIP"),
                                    "Connection failed %s:%d with Code %d", m_host.c_str(), m_port,
                                    resp.status);
                        sleep(10);
                    }
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Connection failed %s:%d",
                                m_host.c_str(), m_port, resp.status);
                }
            }
        }
    }
}

uint8_t NtripClient::write_to_socket(Socket& socket, std::string request_data)
{
    constexpr int flags = MSG_NOSIGNAL;
    auto remaining = static_cast<ssize_t>(request_data.size());
    ssize_t sent = 0;

    while (remaining > 0)
    {
        const auto size =
            ::send(socket, request_data.data() + sent, static_cast<size_t>(remaining), flags);

        if (size < 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Failed to send data %s:%d", m_host.c_str(),
                        m_port);
            break;
        }
        remaining -= size;
        sent += size;
    }
}

Response NtripClient::read_from_socket(Socket& socket, recv_success_t* state,
                                       recv_state_t recv_state)
{
    Response response;
    uint8_t temp_buffer[4096];
    static const uint8_t clrf[] = {'\r', '\n'};
    std::vector<uint8_t> response_data;
    bool first_line = true;
    bool parsed_headers = false;
    int32_t content_size = -1;
    bool chunked_response = false;
    size_t expected_chunk_size = 0;
    bool remove_clrf_after_chunk = false;
    constexpr int32_t flags = MSG_NOSIGNAL;
    *state = RECV_OK;
    /**
     * Continous data, no more header, already received
     **/
    if (recv_state == RECV_CONT)
    {
        parsed_headers = true;
    }

    // read the response
    for (;;)
    {
        const auto size =
            recv(socket, reinterpret_cast<char*>(temp_buffer), sizeof(temp_buffer), flags);
        RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Recieved data, length: %d", size);
        if (size < 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Failed to receive data %s:%d", m_host.c_str(),
                        m_port);
            *state = RECV_FAILED;
            break;
        }
        else if (size == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Connection lost %s:%d", m_host.c_str(),
                        m_port);
            *state = RECV_DISCONNECTED;
            break;  // disconnected
        }
        else
        {
            response_data.insert(response_data.end(), temp_buffer, temp_buffer + size);

            if (!parsed_headers)
            {
                for (;;)
                {
                    const auto i = std::search(response_data.begin(), response_data.end(),
                                               std::begin(clrf), std::end(clrf));

                    // didn't find a newline
                    if (i == response_data.end()) break;

                    const std::string line(response_data.begin(), i);
                    response_data.erase(response_data.begin(), i + 2);

                    // empty line indicates the end of the header section
                    if (line.empty())
                    {
                        parsed_headers = true;
                        break;
                    }
                    else if (first_line)  // first line
                    {
                        first_line = false;

                        std::string::size_type lastPos = 0;
                        const auto length = line.length();
                        std::vector<std::string> parts;

                        // tokenize first line
                        while (lastPos < length + 1)
                        {
                            auto pos = line.find(' ', lastPos);
                            if (pos == std::string::npos) pos = length;

                            if (pos != lastPos)
                                parts.emplace_back(
                                    line.data() + lastPos,
                                    static_cast<std::vector<std::string>::size_type>(pos) -
                                        lastPos);

                            lastPos = pos + 1;
                        }

                        if (parts.size() >= 2) response.status = std::stoi(parts[1]);
                    }
                    else  // headers
                    {
                        response.headers.push_back(line);

                        const auto pos = line.find(':');

                        if (pos != std::string::npos)
                        {
                            std::string headerName = line.substr(0, pos);
                            std::string headerValue = line.substr(pos + 1);

                            // ltrim
                            headerValue.erase(headerValue.begin(),
                                              std::find_if(headerValue.begin(), headerValue.end(),
                                                           [](int c) { return !std::isspace(c); }));

                            // rtrim
                            headerValue.erase(std::find_if(headerValue.rbegin(), headerValue.rend(),
                                                           [](int c) { return !std::isspace(c); })
                                                  .base(),
                                              headerValue.end());

                            if (headerName == "Content-Length")
                                content_size = std::stoi(headerValue);
                            else if (headerName == "Transfer-Encoding" && headerValue == "chunked")
                                chunked_response = true;
                        }
                    }
                }
            }

            if (parsed_headers)
            {
                if (chunked_response)
                {
                    bool dataReceived = false;
                    for (;;)
                    {
                        if (expected_chunk_size > 0)
                        {
                            const auto toWrite =
                                std::min(expected_chunk_size, response_data.size());
                            response.body.insert(
                                response.body.end(), response_data.begin(),
                                response_data.begin() + static_cast<ptrdiff_t>(toWrite));
                            response_data.erase(
                                response_data.begin(),
                                response_data.begin() + static_cast<ptrdiff_t>(toWrite));
                            expected_chunk_size -= toWrite;

                            if (expected_chunk_size == 0) remove_clrf_after_chunk = true;
                            if (response_data.empty()) break;
                        }
                        else
                        {
                            if (remove_clrf_after_chunk)
                            {
                                if (response_data.size() >= 2)
                                {
                                    remove_clrf_after_chunk = false;
                                    response_data.erase(response_data.begin(),
                                                        response_data.begin() + 2);
                                }
                                else
                                    break;
                            }

                            const auto i = std::search(response_data.begin(), response_data.end(),
                                                       std::begin(clrf), std::end(clrf));

                            if (i == response_data.end()) break;

                            const std::string line(response_data.begin(), i);
                            response_data.erase(response_data.begin(), i + 2);

                            expected_chunk_size = std::stoul(line, nullptr, 16);

                            if (expected_chunk_size == 0)
                            {
                                dataReceived = true;
                                break;
                            }
                        }
                    }

                    if (dataReceived) break;
                }
                else
                {
                    response.body.insert(response.body.end(), response_data.begin(),
                                         response_data.end());
                    response_data.clear();

                    // got the whole content
                    if (content_size == -1 ||
                        response.body.size() >= static_cast<size_t>(content_size))
                        break;
                }
            }
        }
    }
    return (response);
}
}  // namespace ntrip