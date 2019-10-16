#include "ubxreceivemessage.hpp"

UbxReceiveMessage::UbxReceiveMessage(UbxMessage::message_class_t class_id,
                                     UbxMessage::message_id_t msg_id, UbxCbInterface* cb)
    : UbxMessage(class_id, msg_id, false)
{
    m_cb = cb;
}

void UbxReceiveMessage::read_message_from(const uint8_t* buffer, const uint16_t length)
{
    UbxMessage::read_message_from(buffer, length);
    if (m_cb != NULL)
    {
        m_cb->message_received(get_identifier(), this);
    }
}
