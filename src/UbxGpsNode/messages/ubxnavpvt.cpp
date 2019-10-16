#include "messages/ubxnavpvt.hpp"

UbxNavPvt::UbxNavPvt(UbxCbInterface* cb) : UbxReceiveMessage(CLASS_NAV, MSG_NAV_PVT, cb) {}
uint8_t UbxNavPvt::get_gps_fix()
{
    uint8_t retVal = read_payload<uint8_t>(20);
    return (retVal);
}
uint8_t UbxNavPvt::get_nav_status_flags1()
{
    uint8_t retVal = read_payload<uint8_t>(21);
    return (retVal);
}
uint8_t UbxNavPvt::get_nav_status_flags2()
{
    uint8_t retVal = read_payload<uint8_t>(22);
    return (retVal);
}

double UbxNavPvt::get_ground_speed()
{
    return (((double)read_payload<int32_t>(60)) / 1000);
}
double UbxNavPvt::get_heading()
{
    return (((double)read_payload<int32_t>(84)) * 1e-5);
}
double UbxNavPvt::get_ground_speed_acc()
{
    return (((double)read_payload<int32_t>(68)) / 1000);
}

double UbxNavPvt::get_heading_acc()
{
    return (((double)read_payload<int32_t>(72)) * 1e-5);
}
