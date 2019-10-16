#pragma once
#include "ubxcfgmessage.hpp"
class UbxUsbProtCfg : public UbxCfgMessage
{
  public:
    UbxUsbProtCfg();
    void set_nmea_output_mode(uint8_t);
    void set_ubx_output_mode(uint8_t);
    void set_rtcm_output_mode(uint8_t);
    static const uint8_t PROT_ON = 1;
    static const uint8_t PROT_OFF = 0;
};
