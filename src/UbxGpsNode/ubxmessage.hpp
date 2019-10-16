#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

/**
 * Message Interface all ubx messages are derived from
 * General Protocoll description can be found under
 * https://gsep.daimler.com/confluence/download/attachments/296455028/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf?api=v2
 */
class UbxMessage
{
   public:
    ///< This decsribes the class of the message
    enum message_class_t
    {
        /// Class of navigation messages
        CLASS_NAV = 0x01,
        /// Class of ack/nack messages
        CLASS_ACK = 0x05,
        /// Class of config messages
        CLASS_CFG = 0x06,

        // marks the class as not set/invalid
        CLASS_INVALID = 0xFF,
    };

    ///< This is the id of the message
    enum message_id_t
    {
        /// nak message id
        MSG_ACK_NAK = 0x00,
        /// ack message id
        MSG_ACK_ACK = 0x01,
        /// Nav status message id
        MSG_NAV_STATUS = 0x03,
        /// Position velocity time message id
        MSG_NAV_PVT = 0x07,
        /// High precission position data message id
        MSG_NAV_HPPOSLLH = 0x14,
        /// End of epoch message id
        MSG_NAV_EOE = 0x61,
        /// Set Value message id
        MSG_VAL_SET = 0x8A,
        // marks the message id as not set/invalid
        MSG_INVALID = 0xFF,
    };

    ///< Combined identifier storing the class id and message id
    struct Identifier
    {
        message_class_t m_class_id;
        message_id_t m_message_id;

        Identifier() : m_class_id(CLASS_INVALID), m_message_id(MSG_INVALID) {}

        Identifier(message_class_t class_id, message_id_t msg_id)
            : m_class_id(class_id), m_message_id(msg_id)
        {
        }

        friend bool operator==(const Identifier& lhs, const Identifier& rhs)
        {
            return lhs.m_class_id == rhs.m_class_id && lhs.m_message_id == rhs.m_message_id;
        }

        static Identifier from_int(uint8_t class_id, uint8_t message_id);
    };

    /**
     * Constructor
     *
     * @param uint8_t class_id class type of mesage (ubx protocol specific)
     * @param uint8_t msg_id message id of mesage (ubx protocol specific)
     * @param const bool is_config flag is true in case this message is a config message
     */
    explicit UbxMessage(message_class_t class_id, message_id_t msg_id, bool is_config);

    /**
     * Reads data from the give position converted to the type T.
     *
     * supported types are:
     *  (u)int8_t, (u)int16_t, (u)int32_t, (u)int64_t, float, double
     * all other types will result in linker errors
     *
     * @param T         type to read
     * @param size_t    offset Byte position to read from
     * @return read value
     */
    template <typename T> T read_payload(size_t offset) const;

    /**
     * Writes data to the give position.
     *
     * supported types are:
     *  (u)int8_t, (u)int16_t, (u)int32_t, (u)int64_t, float, double
     * all other types will result in linker errors
     *
     * @param size_t    offset byte position to write to
     * @param T         value to write
     * @return          size in bytes written
     */
    template <typename T> size_t write_payload(size_t offset, T value);

    /**
     * Returns the class/msg_id combination of the message
     *
     * @return Identifier with the message and class id
     */
    Identifier get_identifier() const
    {
        return Identifier(m_class_id, m_msg_id);
    };

    /**
     *  Returns the size of the entire ubx message in bytes
     */
    size_t get_message_size() const;

    /**
     * Returns the size of the payload
     */
    size_t get_payload_size() const
    {
        return m_payload.size();
    }

    /**
     * Writes the messsage into the passed buffer
     *
     * @param uint8_t* buffer  buffer copied data to
     * @param const uint16_t size  buffer size
     * @return number of copied bytes
     */
    size_t write_message_to(uint8_t* buffer, size_t size) const;

    /**
     * Reads message from byte buffer and initialized this UbxMessage
     * instance with the messages content.
     *
     * @param uint8_t* buffer  buffer copied data from
     * @param const uint16_t length buffer size
     */
    void read_message_from(const uint8_t* buffer, const uint16_t length);

   private:
    /// Class id of message
    message_class_t m_class_id;
    /// Message id
    message_id_t m_msg_id;
    /// Buffer for whole message
    std::vector<uint8_t> m_payload;

    /// Flag to indicate that the message is a config message
    bool m_is_config;
};

// Hash implementation for UbxMessage::Identifier
namespace std
{
template <> struct hash<UbxMessage::Identifier>
{
    typedef UbxMessage::Identifier argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& key) const noexcept
    {
        return static_cast<result_type>((uint16_t)key.m_message_id |
                                        (uint16_t)(key.m_class_id << 8));
    }
};
}  // namespace std
