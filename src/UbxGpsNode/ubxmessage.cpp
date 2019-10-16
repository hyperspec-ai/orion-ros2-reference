#include <endian.h>
#include <cassert>
#include <cstring>
#include <limits>
#include "endianutils.hpp"
#include "ubxmessage.hpp"

// explicit template instantiation
template uint8_t UbxMessage::read_payload<uint8_t>(size_t offset) const;
template uint16_t UbxMessage::read_payload<uint16_t>(size_t offset) const;
template uint32_t UbxMessage::read_payload<uint32_t>(size_t offset) const;
template uint64_t UbxMessage::read_payload<uint64_t>(size_t offset) const;
template int8_t UbxMessage::read_payload<int8_t>(size_t offset) const;
template int16_t UbxMessage::read_payload<int16_t>(size_t offset) const;
template int32_t UbxMessage::read_payload<int32_t>(size_t offset) const;
template int64_t UbxMessage::read_payload<int64_t>(size_t offset) const;

template size_t UbxMessage::write_payload<uint8_t>(size_t offset, uint8_t value);
template size_t UbxMessage::write_payload<uint16_t>(size_t offset, uint16_t value);
template size_t UbxMessage::write_payload<uint32_t>(size_t offset, uint32_t value);
template size_t UbxMessage::write_payload<uint64_t>(size_t offset, uint64_t value);
template size_t UbxMessage::write_payload<int8_t>(size_t offset, int8_t value);
template size_t UbxMessage::write_payload<int16_t>(size_t offset, int16_t value);
template size_t UbxMessage::write_payload<int32_t>(size_t offset, int32_t value);
template size_t UbxMessage::write_payload<int64_t>(size_t offset, int64_t value);

// Ubx message header
struct UbxMessageHeader
{
    UbxMessageHeader(UbxMessage::message_class_t class_id, UbxMessage::message_id_t message_id,
                     uint16_t payload_size)
        : m_sync1(SYNC_CHAR_1),
          m_sync2(SYNC_CHAR_2),
          m_class_id(class_id),
          m_message_id(message_id),
          m_payload_size(payload_size)
    {
    }

    UbxMessage::message_class_t get_message_class() const
    {
        return static_cast<UbxMessage::message_class_t>(m_class_id);
    }

    UbxMessage::message_id_t get_message_id() const
    {
        return static_cast<UbxMessage::message_id_t>(m_message_id);
    }

    uint16_t get_payload_size() const
    {
        return m_payload_size;
    }

   private:
    const static uint8_t SYNC_CHAR_1 = 0xb5;
    const static uint8_t SYNC_CHAR_2 = 0x62;

    uint8_t m_sync1;          // sync char, must be set to SYNC_CHAR_1
    uint8_t m_sync2;          // sync char, must be set to SYNC_CHAR_2
    uint8_t m_class_id;       // message class id (see UbxMessage::message_class_t)
    uint8_t m_message_id;     // message id (see UbxMessage::message_id_t)
    uint16_t m_payload_size;  // size of the payload data in bytes

} __attribute__((packed));

static_assert(sizeof(UbxMessageHeader) == 6, "UbxMessageHeader should be 6 bytes in size");

UbxMessage::Identifier UbxMessage::Identifier::from_int(uint8_t class_id, uint8_t message_id)
{
    const message_class_t conv_class_id = static_cast<UbxMessage::message_class_t>(class_id);
    const message_id_t conv_msg_id = static_cast<UbxMessage::message_id_t>(message_id);

    assert(conv_class_id == CLASS_ACK || conv_class_id == CLASS_CFG || conv_class_id == CLASS_NAV);
    assert(conv_msg_id == MSG_ACK_ACK || conv_msg_id == MSG_ACK_NAK || conv_msg_id == MSG_NAV_EOE ||
           conv_msg_id == MSG_NAV_HPPOSLLH || conv_msg_id == MSG_NAV_PVT ||
           conv_msg_id == MSG_NAV_STATUS || conv_msg_id == MSG_VAL_SET);

    return Identifier(conv_class_id, conv_msg_id);
}

// This holds the fletcher CRC hash, which is stored in two bytes.
struct CRCFletcher8
{
    CRCFletcher8() : ck_a(0), ck_b(0) {}

    uint8_t ck_a;
    uint8_t ck_b;
};

// calculate the 8-bit fletcher CRC for the specific data
CRCFletcher8 calculate_fletcher8_crc(uint8_t* data, size_t data_size)
{
    CRCFletcher8 crc;

    for (size_t i = 0; i < data_size; ++i)
    {
        crc.ck_a = crc.ck_a + data[i];
        crc.ck_b = crc.ck_b + crc.ck_a;
    }

    return crc;
}

UbxMessage::UbxMessage(UbxMessage::message_class_t class_id, UbxMessage::message_id_t msg_id,
                       const bool is_config)
{
    m_class_id = class_id;
    m_msg_id = msg_id;
    m_is_config = is_config;
}

size_t UbxMessage::write_message_to(uint8_t* buffer, size_t size) const
{
    assert(buffer != nullptr);
    assert(size > 0);

    const size_t payload_size = m_payload.size();
    const size_t header_size = sizeof(UbxMessageHeader);
    const size_t footer_size = 2;  // 2 bytes for checksum
    const size_t message_size = header_size + payload_size + footer_size;

    assert(size >= payload_size + header_size);
    assert(payload_size <= std::numeric_limits<uint16_t>::max());

    // write message header into the destination buffer
    UbxMessageHeader* header = (UbxMessageHeader*)buffer;
    *header = UbxMessageHeader(m_class_id, m_msg_id, payload_size);

    // write the payload into the destination buffer
    memcpy(&buffer[header_size], m_payload.data(), payload_size);

    // calculate crc and attach it to the end of the buffer
    CRCFletcher8 crc = calculate_fletcher8_crc(&buffer[2], payload_size + header_size - 2);

    const size_t crc_offset = header_size + payload_size;
    buffer[crc_offset + 0] = crc.ck_a;
    buffer[crc_offset + 1] = crc.ck_b;

    return message_size;
}

void UbxMessage::read_message_from(const uint8_t* buffer, const uint16_t size)
{
    assert(buffer);

    const size_t header_size = sizeof(UbxMessageHeader);
    const size_t footer_size = 2;

    UbxMessageHeader* header = (UbxMessageHeader*)buffer;

    assert((header->get_payload_size() + header_size + footer_size) == size);

    m_class_id = header->get_message_class();
    m_msg_id = header->get_message_id();

    m_payload.resize(header->get_payload_size());
    memcpy(m_payload.data(), &buffer[header_size], header->get_payload_size());
}

template <typename T> T UbxMessage::read_payload(size_t offset) const
{
    assert(offset + sizeof(T) <= m_payload.size());
    return EndianUtils::letoh(*(T*)(&m_payload[offset]));
}

template <> float UbxMessage::read_payload<float>(size_t offset) const
{
    uint32_t val = read_payload<uint32_t>(offset);
    return *(float*)&val;
}

template <> double UbxMessage::read_payload<double>(size_t offset) const
{
    uint64_t val = read_payload<uint64_t>(offset);
    return *(double*)&val;
}

template <typename T> size_t UbxMessage::write_payload(size_t offset, T value)
{
    // when sending config values, the alignment are different. config does not
    // have any alignment or padding requirements
    if (!m_is_config)
    {
        // ensure alignment is according to UBX specs (alignment must be at least data size) for
        // non-config payload
        assert((offset % sizeof(value)) == 0);
    }

    // resize payload if needed
    const size_t required_size = offset + sizeof(T);
    if (m_payload.size() < required_size)
    {
        m_payload.resize(required_size);
    }

    // write payload
    assert(offset + sizeof(T) <= m_payload.size());
    *(T*)(&m_payload[offset]) = EndianUtils::htole(value);
    return sizeof(value);
}

template <> size_t UbxMessage::write_payload<float>(size_t offset, float value)
{
    return write_payload<uint32_t>(offset, *(uint32_t*)&value);
}

template <> size_t UbxMessage::write_payload<double>(size_t offset, double value)
{
    return write_payload<uint64_t>(offset, *(uint64_t*)&value);
}

size_t UbxMessage::get_message_size() const
{
    const size_t payload_size = m_payload.size();
    const size_t header_size = sizeof(UbxMessageHeader);
    const size_t footer_size = 2;  // 2 bytes for checksum

    return header_size + payload_size + footer_size;
}
