#include "messages/ubxusbmsgcfg.hpp"
#include <cassert>

UbxUsbMsgCfg::UbxUsbMsgCfg() : UbxCfgMessage(CLASS_CFG, MSG_VAL_SET)
{
    set_layer(UbxCfgMessage::LAYER_RAM);
    set_highprecission_msg(1);
    set_status_msg(1);
    set_EOE_msg(1);
    set_pvg_msg(1);
    assert(get_payload_size() == 24);
}

void UbxUsbMsgCfg::set_highprecission_msg(uint8_t mode)
{
    write_payload<uint32_t>(4, 0x20910036);
    write_payload<uint8_t>(8, mode);
}

void UbxUsbMsgCfg::set_status_msg(uint8_t mode)
{
    write_payload<uint32_t>(9, 0x2091001d);
    write_payload<uint8_t>(13, mode);
}

void UbxUsbMsgCfg::set_EOE_msg(uint8_t mode)
{
    write_payload<uint32_t>(14, 0x20910162);
    write_payload<uint8_t>(18, mode);
}

void UbxUsbMsgCfg::set_pvg_msg(uint8_t mode)
{
    write_payload<uint32_t>(19, 0x20910009);
    write_payload<uint8_t>(23, mode);
}
