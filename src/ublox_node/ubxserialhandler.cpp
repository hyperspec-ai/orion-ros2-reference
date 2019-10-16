#include <cassert>
#include "ubxserialhandler.hpp"

bool UbxSerialHandler::enqueue_ubx_cfg_message(UbxMessage* msg)
{
    assert(msg != nullptr);
    m_config_messages.push_back(msg);
    return (true);
}
UbxSerialHandler::UbxSerialHandler(const std::string& port) : m_ack_msg(this), m_nak_msg(this)
{
    m_port_name = port;
    m_running = true;
    m_cfg_send_state = CFG_STATE_IDLE;
    register_receive_msg(&m_ack_msg);
    register_receive_msg(&m_nak_msg);
}

UbxSerialHandler::~UbxSerialHandler()
{
    /*
    for (auto msg : m_receive_messages)
    {
        delete msg.second;
    }
    */

    for (auto msg : m_config_messages)
    {
        delete msg;
    }
}

bool UbxSerialHandler::register_receive_msg(UbxReceiveMessage* msg)
{
    m_receive_messages[msg->get_identifier()] = msg;
    return (true);
}
void UbxSerialHandler::close_port()
{
    m_running = false;
    m_read_thread->join();
    m_config_thread->join();
    close(m_port);
}
bool UbxSerialHandler::init_comport()
{
    bool ret_val = true;
    m_port = open(m_port_name.c_str(), O_RDWR);
    if (m_port != -1)
    {
        termios tty = get_termios();
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag &= ~PARENB;   // No parity bit is added to the output characters
        tty.c_cflag &= ~CSTOPB;   // Only one stop-bit is used
        tty.c_cflag &= ~CSIZE;    // CSIZE is a mask for the number of bits per character
        tty.c_cflag |= CS8;       // Set to 8 bits per character
        tty.c_cflag &= ~CRTSCTS;  // Disable hadrware flow control (RTS/CTS)
        tty.c_cflag |= CREAD | CLOCAL;
        // tty.c_oflag = 0; // No remapping, no delays
        // tty.c_oflag &= ~OPOST; // Make raw
        tty.c_cc[VTIME] = (cc_t)(10);  // 1 second read timeout
        tty.c_cc[VMIN] = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_lflag &= ~ICANON;  // Turn off canonical input, which is suitable for pass-through
        tty.c_lflag &= ~ECHO;    // Configure echo depending on echo_ boolean
        tty.c_lflag &=
            ~ECHOE;  // Turn off echo erase (echo erase only relevant if canonical input is active)
        tty.c_lflag &= ~ECHONL;  //
        tty.c_lflag &= ~ISIG;
        set_termios(tty);
        // spdlog::info("UBX Port opened succesfully, starting worker threads");
        m_read_thread = new std::thread([this] { receive_message_thread(); });
        m_config_thread = new std::thread([this] { send_thread(); });
    }
    else
    {
        ret_val = false;
    }
    return (ret_val);
}

termios UbxSerialHandler::get_termios()
{
    if (m_port == -1)
        throw std::runtime_error("get_termios() called but file descriptor was not valid.");

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Get current settings (will be stored in termios structure)
    if (tcgetattr(m_port, &tty) != 0)
    {
        // Error occurred
        std::cout << "Could not get terminal attributes for \"" << m_port_name << "\" - "
                  << strerror(errno) << std::endl;
        throw std::system_error(EFAULT, std::system_category());
        // return false;
    }

    return tty;
}

void UbxSerialHandler::set_termios(termios term)
{
    // Flush port, then apply attributes
    tcflush(m_port, TCIFLUSH);

    if (tcsetattr(m_port, TCSANOW, &term) != 0)
    {
        // Error occurred
        std::cout << "Could not apply terminal attributes for \"" << m_port_name << "\" - "
                  << strerror(errno) << std::endl;
        throw std::system_error(EFAULT, std::system_category());
    }

    // Successful!
}
ssize_t UbxSerialHandler::get_ser_byte(uint8_t* buffer, uint16_t len)
{
    ssize_t n = read(m_port, buffer, len);
    // spdlog::info("Data received:{0}", n);
    return (n);
}
void UbxSerialHandler::receive_message_thread()
{
    uint8_t buffer[1024];
    uint8_t* buffer_pointer = &buffer[0];
    uint8_t read_length = STATE_KEY_1_LENGTH;

    UbxMessage::Identifier msg_id;

    uint16_t length;
    while (m_running)
    {
        if (get_ser_byte(buffer_pointer, read_length) == read_length)
        {
            switch (m_receive_state)
            {
                case STATE_WAITING_KEY_1:
                    if (buffer_pointer[0] == KEY_1)
                    {
                        // spdlog::trace("UBX KEY_1 Received");
                        m_receive_state = STATE_WAITING_KEY_2;
                        read_length = STATE_KEY_2_LENGTH;
                        buffer_pointer += STATE_KEY_1_LENGTH;
                    }
                    break;
                case STATE_WAITING_KEY_2:
                    if (buffer_pointer[0] == KEY_2)
                    {
                        // spdlog::trace("UBX KEY_2 Received");
                        m_receive_state = STATE_WAITING_MSG_ID;
                        read_length = STATE_MSG_ID_LENGTH;
                        buffer_pointer += STATE_KEY_2_LENGTH;
                    }
                    else
                    {
                        // spdlog::trace("UBX KEY_2 wrong, start new try{0}", buffer_pointer[0]);
                        m_receive_state = STATE_WAITING_KEY_1;
                        read_length = STATE_KEY_1_LENGTH;
                        buffer_pointer = &buffer[0];
                    }
                    break;

                case STATE_WAITING_MSG_ID:
                    msg_id = UbxMessage::Identifier::from_int(buffer_pointer[0], buffer_pointer[1]);

                    // spdlog::trace("UBX Msg ID Received {0:x}", msg_id);
                    m_receive_state = STATE_WAITING_LENGTH;
                    read_length = STATE_LENGTH_LENGTH;
                    buffer_pointer += STATE_MSG_ID_LENGTH;
                    break;

                case STATE_WAITING_LENGTH:
                    length = (((uint16_t)buffer_pointer[1]) << 8) | ((uint16_t)buffer_pointer[0]);
                    // spdlog::trace("UBX Length Received {0}", length);
                    m_receive_state = STATE_WAITING_DATA;
                    buffer_pointer += STATE_LENGTH_LENGTH;
                    read_length = length;
                    break;

                case STATE_WAITING_DATA:
                    // spdlog::trace("UBX Data Received");
                    m_receive_state = STATE_WAITING_CS;
                    buffer_pointer += length;
                    read_length = STATE_CS_LENGTH;
                    break;

                case STATE_WAITING_CS:
                    if (m_receive_messages.count(msg_id) != 0)
                    {
                        m_receive_messages[msg_id]->read_message_from(buffer, length + 8);
                    }
                    else
                    {
                    }

                    m_receive_state = STATE_WAITING_KEY_1;
                    buffer_pointer = &buffer[0];
                    read_length = STATE_KEY_1_LENGTH;
                    break;
            }
        }
        else
        {
        }
    }
}

void UbxSerialHandler::message_received(UbxMessage::Identifier id, const UbxMessage* /*msg*/)
{
    if ((id == m_ack_msg.get_identifier()) && (m_cfg_send_state == CFG_STATE_SEND))
    {
        m_cfg_send_state = CFG_STATE_RECVD;
    }
    else if (id == m_nak_msg.get_identifier())
    {
        m_cfg_send_state = CFG_STATE_ERROR;
    }
}
void UbxSerialHandler::add_rtcm_message(std::vector<uint8_t>& msg)
{
    std::lock_guard<std::mutex> lock(m_rtcm_mutex);
    m_rtcm_messages.push(msg);
}

void UbxSerialHandler::send_thread()
{
    std::vector<uint8_t> data_buffer;

    // Configure first....
    for (auto msg : m_config_messages)
    {
        data_buffer.resize(msg->get_message_size());

        size_t len = msg->write_message_to(data_buffer.data(), data_buffer.size());
        assert(len == data_buffer.size());

        size_t write_result = write(m_port, data_buffer.data(), len);
        assert(write_result == len);

        m_cfg_send_state = CFG_STATE_SEND;
        while (m_cfg_send_state == CFG_STATE_SEND)
        {
            usleep(10000);
            if (m_running != true)
            {
                break;
            }
        }
        if (m_cfg_send_state != CFG_STATE_RECVD)
        {
        }
        else
        {
        }
        if (m_running != true)
        {
            break;
        }
    }
    while (m_running)
    {
        {
            std::lock_guard<std::mutex> lock(m_rtcm_mutex);
            while (!m_rtcm_messages.empty())
            {
                std::vector<uint8_t> msg = m_rtcm_messages.front();
                write(m_port, msg.data(), msg.size());
                m_rtcm_messages.pop();
            }
        }
        sleep(1);
    }
}
