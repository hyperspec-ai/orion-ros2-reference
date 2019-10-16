#pragma once
#include "ubxcfgmessage.hpp"
class UbxUsbMsgCfg : public UbxCfgMessage
{
  public:
    UbxUsbMsgCfg();
    void set_highprecission_msg(uint8_t);
    void set_status_msg(uint8_t);
    void set_pvg_msg(uint8_t);
    void set_EOE_msg(uint8_t mode);
    static const uint8_t MSG_OFF = 0;
};