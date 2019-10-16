#pragma once
#include "ubxreceivemessage.hpp"
class UbxNavHighPresGeo : public UbxReceiveMessage
{
  private:
    /* data */
  public:
    UbxNavHighPresGeo(UbxCbInterface* cb);
    uint8_t get_version();
    double get_lon();
    double get_lat();
    double get_horicontal_acc();
    double get_vertical_acc();
    double get_height();
};
