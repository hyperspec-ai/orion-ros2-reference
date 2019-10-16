
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include "rclcpp/rclcpp.hpp"
#include "socket.hpp"
namespace ntrip
{
constexpr int get_address_family(InternetProtocol internetProtocol)
{
    return (internetProtocol == InternetProtocol::V4)
               ? AF_INET
               : (internetProtocol == InternetProtocol::V6)
                     ? AF_INET6
                     : throw std::runtime_error("Unsupported protocol");
}

Socket::Socket(InternetProtocol internetProtocol)
    : endpoint(socket(get_address_family(internetProtocol), SOCK_STREAM, IPPROTO_TCP))
{
    if (endpoint == INVALID)
    {
        RCLCPP_INFO(rclcpp::get_logger("NTRIP"), "Socket creation failed");
    }
}

Socket::Socket(Type s) noexcept : endpoint(s) {}

Socket::~Socket()
{
    if (endpoint != INVALID) close();
}

Socket::Socket(Socket&& other) noexcept : endpoint(other.endpoint)
{
    other.endpoint = INVALID;
}

Socket& Socket::operator=(Socket&& other) noexcept
{
    if (&other != this)
    {
        if (endpoint != INVALID) close();
        endpoint = other.endpoint;
        other.endpoint = INVALID;
    }

    return *this;
}

inline void Socket::close() noexcept
{
    ::close(endpoint);
}
}  // namespace ntrip