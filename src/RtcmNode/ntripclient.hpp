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
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
#include "base64/base64.h"
#include "ntripcallback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket.hpp"
namespace ntrip
{
/*! Response structure for reading data from the http connection*/
struct Response final
{
    /*Status Codes from http*/
    enum Status
    {
        STATUS_CONTINUE = 100,
        STATUS_SWITCHINGPROTOCOLS = 101,
        STATUS_PROCESSING = 102,
        STATUS_EARLYHINTS = 103,

        STATUS_OK = 200,
        STATUS_CREATED = 201,
        STATUS_ACCEPTED = 202,
        STATUS_NONAUTHORITATIVEINFORMATION = 203,
        STATUS_NOCONTENT = 204,
        STATUS_RESETCONTENT = 205,
        STATUS_PARTIALCONTENT = 206,
        STATUS_MULTISTATUS = 207,
        STATUS_ALREADYREPORTED = 208,
        STATUS_IMUSED = 226,

        STATUS_MULTIPLECHOICES = 300,
        STATUS_MOVEDPERMANENTLY = 301,
        STATUS_FOUND = 302,
        STATUS_SEEOTHER = 303,
        STATUS_NOTMODIFIED = 304,
        STATUS_USEPROXY = 305,
        STATUS_TEMPORARYREDIRECT = 307,
        STATUS_PERMANENTREDIRECT = 308,

        STATUS_BADREQUEST = 400,
        STATUS_UNAUTHORIZED = 401,
        STATUS_PAYMENTREQUIRED = 402,
        STATUS_FORBIDDEN = 403,
        STATUS_NOTFOUND = 404,
        STATUS_METHODNOTALLOWED = 405,
        STATUS_NOTACCEPTABLE = 406,
        STATUS_PROXYAUTHENTICATIONREQUIRED = 407,
        STATUS_REQUESTTIMEOUT = 408,
        STATUS_CONFLICT = 409,
        STATUS_GONE = 410,
        STATUS_LENGTHREQUIRED = 411,
        STATUS_PRECONDITIONFAILED = 412,
        STATUS_PAYLOADTOOLARGE = 413,
        STATUS_URITOOLONG = 414,
        STATUS_UNSUPPORTEDMEDIATYPE = 415,
        STATUS_RANGENOTSATISFIABLE = 416,
        STATUS_EXPECTATIONFAILED = 417,
        STATUS_IMATEAPOT = 418,
        STATUS_MISDIRECTEDREQUEST = 421,
        STATUS_UNPROCESSABLEENTITY = 422,
        STATUS_LOCKED = 423,
        STATUS_FAILEDDEPENDENCY = 424,
        STATUS_TOOEARLY = 425,
        STATUS_UPGRADEREQUIRED = 426,
        STATUS_PRECONDITIONREQUIRED = 428,
        STATUS_TOOMANYREQUESTS = 429,
        STATUS_REQUESTHEADERFIELDSTOOLARGE = 431,
        STATUS_UNAVAILABLEFORLEGALREASONS = 451,

        STATUS_INTERNALSERVERERROR = 500,
        STATUS_NOTIMPLEMENTED = 501,
        STATUS_BADGATEWAY = 502,
        STATUS_SERVICEUNAVAILABLE = 503,
        STATUS_GATEWAYTIMEOUT = 504,
        STATUS_HTTPVERSIONNOTSUPPORTED = 505,
        STATUS_VARIANTALSONEGOTIATES = 506,
        STATUS_INSUFFICIENTSTORAGE = 507,
        STATUS_LOOPDETECTED = 508,
        STATUS_NOTEXTENDED = 510,
        STATUS_NETWORKAUTHENTICATIONREQUIRED = 511
    };
    /*!Returned state (only valid in case of starting communication)*/
    int32_t status = 0;
    /*!Returned headers (only valid in case of starting communication)*/
    std::vector<std::string> headers;
    /*!Returned data*/
    std::vector<uint8_t> body;
};
/*! Ntrip Client implementation*/

class NtripClient
{
   public:
    /**
     * Constructor
     *
     * @param onst std::string& host host name or ip address
     * @param uint16_t port Port
     * @param const std::string& mount_point NTRIP Mountpoint
     * @param NtripCallback* callback Callbacj for received RTCM sentences
     * @param const std::string& user User Name for ntrip server, only base Auth is supported
     * @param onst std::string& password Password for ntrip server, only base Auth is supported
     */
    NtripClient(const std::string& host, uint16_t port, const std::string& mount_point,
                NtripCallback* callback, const std::string& user, const std::string& password);

    /**
     * Shutting down ntrip server
     **/
    void shutdown();
    /**
     * Update gga NMEA sentence / position information to update VRS position
     * @param std::string gga GPGGA NMEA Sentence of the current position
     **/
    void set_gga(std::string gga);

   private:
    enum recv_state_t
    {
        RECV_START = 0,
        RECV_CONT = 1
    };
    enum recv_success_t
    {
        RECV_OK = 0,
        RECV_FAILED = 1,
        RECV_DISCONNECTED = 2
    };
    /**
     * Receive Thread started by constructor and shut down with call of shutdown()
     **/
    void worker_thread();
    /**
     * Write data to socket
     * @param Socket& socket Socket data to be send
     * @param std::string requestData data to be send
     **/
    uint8_t write_to_socket(Socket& socket, std::string requestData);
    /**
     * Reads data from socket
     * @param Socket& socket socket to read from
     * @param uint8_t* state returns the staus of the reas operation
     * @param recv_state_t rstate in case of RECV_START http header is read and analysed
     **/
    Response read_from_socket(Socket& socket, recv_success_t* state, recv_state_t rstate);
    /*!GGA NMEA Sentence of current situation, initial Position is Berlin Tiergarten*/
    std::string m_gga = "$GPGGA,144434.860,5230.791,N,01321.387,E,1,12,1.0,0.0,M,0.0,M,,*64\r\n";
    /*Pointer to thread object for worker_thread()*/
    std::thread* m_thread;

    std::string m_host;
    uint16_t m_port;
    std::string m_auth;
    std::string m_mount_point;
    NtripCallback* m_callback;
    std::atomic<bool> m_running;
    std::mutex m_gga_mutex;
};
}  // namespace ntrip