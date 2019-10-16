#include "messages/ubxnavstatus.hpp"

UbxNavStatus::UbxNavStatus(UbxCbInterface* cb) : UbxReceiveMessage(CLASS_NAV, MSG_NAV_STATUS, cb) {}
uint8_t UbxNavStatus::get_gps_fix()
{
    uint8_t retVal = read_payload<uint8_t>(4);
    return (retVal);
}
uint8_t UbxNavStatus::get_nav_status_flags1()
{
    uint8_t retVal = read_payload<uint8_t>(5);
    return (retVal);
}
uint8_t UbxNavStatus::get_nav_status_flags2()
{
    uint8_t retVal = read_payload<uint8_t>(7);
    return (retVal);
}
uint8_t UbxNavStatus::get_fix_status()
{
    uint8_t retVal = read_payload<uint8_t>(6);
    return (retVal);
}
uint32_t UbxNavStatus::get_time_to_first()
{
    uint32_t retVal = read_payload<uint8_t>(8);
    return (retVal);
}
uint32_t UbxNavStatus::time_since_start_up()
{
    uint32_t retVal = read_payload<uint8_t>(12);
    return (retVal);
}