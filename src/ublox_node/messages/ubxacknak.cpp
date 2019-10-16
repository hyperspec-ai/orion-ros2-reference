#include "messages/ubxacknak.hpp"

UbxAckNak::UbxAckNak(UbxCbInterface* cb) : UbxReceiveMessage(CLASS_ACK, MSG_ACK_NAK, cb) {}

uint16_t UbxAckNak::get_received_msg_id()
{
    uint16_t retVal = 0;
    uint8_t class_id = (uint8_t)read_payload<uint8_t>(0);
    uint8_t msg_id = (uint8_t)read_payload<uint8_t>(1);
    retVal = (((uint16_t)class_id) << 8) | ((uint16_t)msg_id);
    return (retVal);
}