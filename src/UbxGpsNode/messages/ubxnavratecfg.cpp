#include "messages/ubxnavratecfg.hpp"
#include <cassert>

UbxNavRateCfg::UbxNavRateCfg() : UbxCfgMessage(CLASS_CFG, MSG_VAL_SET)
{
    set_meas_rate_ms(1000);
    set_rate_time_ref(TIMEREF_GPS);
    set_dyn_model(DYNMODE_AUTOMOTIVE);
    set_layer(UbxCfgMessage::LAYER_RAM);

    assert(get_payload_size() == 26);
}

void UbxNavRateCfg::set_rate_nav(uint16_t rate)
{
    write_payload<uint32_t>(4, 0x30210002);
    write_payload<uint16_t>(8, rate);
}

void UbxNavRateCfg::set_meas_rate_ms(uint16_t rate)
{
    write_payload<uint32_t>(10, 0x30210001);
    write_payload<uint16_t>(14, rate);
}

void UbxNavRateCfg::set_rate_time_ref(uint8_t mode)
{
    write_payload<uint32_t>(16, 0x20210003);
    write_payload<uint8_t>(20, mode);
}

void UbxNavRateCfg::set_dyn_model(uint8_t mode)
{
    write_payload<uint32_t>(21, 0x20110021);
    write_payload<uint8_t>(25, mode);
}
