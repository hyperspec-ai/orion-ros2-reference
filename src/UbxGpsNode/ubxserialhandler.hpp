#pragma once
#include <errno.h>    // Error number definitions
#include <fcntl.h>    // File control definitions
#include <stdio.h>    // Standard input/output definitions
#include <string.h>   // String function definitions
#include <termios.h>  // POSIX terminal control definitions (struct termios)
#include <unistd.h>   // UNIX standard function definitions
#include <atomic>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <system_error>  // For throwing std::system_error
#include <thread>
#include <unordered_map>
#include <vector>
#include "messages/ubxackack.hpp"
#include "messages/ubxacknak.hpp"
#include "ubxmessage.hpp"
#include "ubxreceivemessage.hpp"
/**
 * Serial Interface handler for ubx devices
 **/
class UbxSerialHandler : public UbxCbInterface
{
   public:
    /**
     * Constructor
     *
     * @param const std::string& port path to serial port /dev/tty etc.
     * @param const std::string& path_to_str2str path to rtklib stream 2 stream client
     * @param const std::string& ntrip_parameter parameter to start str2str
     */
    explicit UbxSerialHandler(const std::string& port);

    virtual ~UbxSerialHandler();

    /**
     * Function used to enque config messages before communication is started. This transfers
     * ownership of the message memory to the UbxSerialHandler.
     *
     * @param const UbxMessage msg config message to be send
     */
    bool enqueue_ubx_cfg_message(UbxMessage* msg);
    /**
     * Function to register a specifc message to be received
     *
     * @param const UbxMessage msg recieve message to be send
     */
    bool register_receive_msg(UbxReceiveMessage* msg);
    /**
     * Init COM port and start reception
     *
     * @return true in case init was successfull
     */
    bool init_comport();
    /**
     * Call back for config message handling, defined by UbxReceiveMessage
     */
    void message_received(UbxMessage::Identifier id, const UbxMessage* msg);
    /**
     * Kills all str2str instances and restart
     */
    void restart_str2str();
    /**
     * Close ports and threads
     */
    void close_port();
    /**
     * Close ports and threads
     */
    void add_rtcm_message(std::vector<uint8_t>& msg);

   private:
    /// Serial port path
    std::string m_port_name;
    /// Control flag to shutdown threads
    std::atomic<bool> m_running;
    /// map of registered receive messages
    std::unordered_map<UbxMessage::Identifier, UbxReceiveMessage*> m_receive_messages;
    /// list of configuration messages to be send on communication start
    std::vector<UbxMessage*> m_config_messages;
    /// message queue for rtcm correction data messages
    std::queue<std::vector<uint8_t>> m_rtcm_messages;
    /// mutex to protect rtcm message queue
    std::mutex m_rtcm_mutex;
    /// path to str2str binary from RTKLIB
    FILE* m_handle;
    /// Comport handle
    int m_port;
    /// Thread handling recpetion of data, build of messages and distribute messages to register
    /// callback
    void receive_message_thread();
    /// Thread for str2str start
    ssize_t get_ser_byte(uint8_t* buffer, uint16_t len);
    /// Thread caring about sending config messages etc
    void send_thread();
    /// Getting terminos data of serial port
    termios get_termios();
    /// Setting terminos data of serial port
    void set_termios(termios term);
    /// buffer index for serial receive buffer (write)
    uint32_t m_buffer_fill = 0;
    /// buffer index for serial receive buffer (read)
    uint32_t m_read_pointer = 0;
    /// Serial receive buffer
    uint8_t m_ser_buff[100];
    /// Ubx specific state machine states
    static const uint8_t STATE_WAITING_KEY_1 = 0x00;
    static const uint8_t STATE_WAITING_KEY_2 = 0x01;
    static const uint8_t STATE_WAITING_MSG_ID = 0x02;
    static const uint8_t STATE_WAITING_LENGTH = 0x03;
    static const uint8_t STATE_WAITING_DATA = 0x04;
    static const uint8_t STATE_WAITING_CS = 0x05;

    static const uint8_t STATE_KEY_1_LENGTH = 0x01;
    static const uint8_t STATE_KEY_2_LENGTH = 0x01;
    static const uint8_t STATE_MSG_ID_LENGTH = 0x02;
    static const uint8_t STATE_LENGTH_LENGTH = 0x02;
    static const uint8_t STATE_CS_LENGTH = 0x02;

    static const uint8_t CFG_STATE_IDLE = 0x00;
    static const uint8_t CFG_STATE_SEND = 0x01;
    static const uint8_t CFG_STATE_RECVD = 0x02;
    static const uint8_t CFG_STATE_ERROR = 0x03;
    /// Header Key 1
    static const uint8_t KEY_1 = 0xB5;
    /// Header Key 2
    static const uint8_t KEY_2 = 0x62;
    /// Message neccesray for config process
    UbxAckAck m_ack_msg;
    UbxAckNak m_nak_msg;
    /// Recieve state machine state
    uint8_t m_receive_state = STATE_WAITING_KEY_1;
    /// Configuration state of current config message
    std::atomic<uint8_t> m_cfg_send_state;
    /// Thread deffinitions
    std::thread* m_read_thread;
    std::thread* m_config_thread;
};
