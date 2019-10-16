#pragma once
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <system_error>
namespace ntrip
{
/*! Enums for IP versions*/
enum class InternetProtocol : uint8_t
{
    V4,
    V6
};
/*Check ip version*/
constexpr int get_address_family(InternetProtocol internetProtocol);

/*! Socket Wrapper*/
class Socket final
{
   public:
    using Type = int;
    static constexpr Type INVALID = -1;

    explicit Socket(InternetProtocol internetProtocol);
    explicit Socket(Type s) noexcept;

    ~Socket();

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    Socket(Socket&& other) noexcept;

    Socket& operator=(Socket&& other) noexcept;

    inline operator Type() const noexcept
    {
        return endpoint;
    }

   private:
    inline void close() noexcept;
    Type endpoint = INVALID;
};
}  // namespace ntrip