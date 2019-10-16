#include "messages/ubxnaveoe.hpp"

UbxNavEoe::UbxNavEoe(UbxCbInterface* cb) : UbxReceiveMessage(CLASS_NAV, MSG_NAV_EOE, cb) {}

uint32_t UbxNavEoe::get_itow()
{
    uint32_t retVal = 0;
    retVal = read_payload<uint32_t>(0);
    return (retVal);
}