#include "messages/ubxusbprotcfg.hpp"
#include <cassert>

UbxUsbProtCfg::UbxUsbProtCfg() : UbxCfgMessage(CLASS_CFG, MSG_VAL_SET)
{
    set_layer(UbxCfgMessage::LAYER_RAM);
    set_nmea_output_mode(PROT_ON);
    set_ubx_output_mode(PROT_OFF);
    set_rtcm_output_mode(PROT_OFF);

    assert(get_payload_size() == 19);
}

void UbxUsbProtCfg::set_nmea_output_mode(uint8_t mode)
{
    write_payload<uint32_t>(4, 0x10780002);
    write_payload<uint8_t>(8, mode);
}

void UbxUsbProtCfg::set_ubx_output_mode(uint8_t mode)
{
    write_payload<uint32_t>(9, 0x10780001);
    write_payload<uint8_t>(13, mode);
}

void UbxUsbProtCfg::set_rtcm_output_mode(uint8_t mode)
{
    write_payload<uint32_t>(14, 0x10780004);
    write_payload<uint8_t>(18, mode);
}
