#include "ubxcfgmessage.hpp"

void UbxCfgMessage::set_layer(const uint8_t layer)
{
    /**Message Version 1 */
    write_payload<uint8_t>(0, 1);
    write_payload<uint8_t>(1, (uint32_t)layer);
    write_payload<uint16_t>(2, 0);
}

UbxCfgMessage::UbxCfgMessage(UbxMessage::message_class_t class_id, UbxMessage::message_id_t msg_id)
    : UbxMessage(class_id, msg_id, true)
{
}
