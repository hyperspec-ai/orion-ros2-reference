#include "messages/ubxnavhighpresgeo.hpp"

UbxNavHighPresGeo::UbxNavHighPresGeo(UbxCbInterface* cb)
    : UbxReceiveMessage(CLASS_NAV, MSG_NAV_HPPOSLLH, cb)
{
}

uint8_t UbxNavHighPresGeo::get_version()
{
    uint8_t retVal = read_payload<uint8_t>(0);
    return (retVal);
}

double UbxNavHighPresGeo::get_height()
{
    uint32_t height = read_payload<int32_t>(16);
    return ((double)height) / 1000.0;
}

double UbxNavHighPresGeo::get_lat()
{
    uint32_t lat = read_payload<int32_t>(12);
    uint32_t lat_hp = read_payload<int8_t>(25);
    return ((((double)lat) * 1e-7) + (((double)lat_hp) * 1e-9));
}

double UbxNavHighPresGeo::get_lon()
{
    int32_t lon = read_payload<int32_t>(8);
    int8_t lon_hp = read_payload<int8_t>(24);
    double retVal = ((((double)lon) * 1e-7) + (((double)lon_hp) * 1e-9));
    return (retVal);
}

double UbxNavHighPresGeo::get_vertical_acc()
{
    uint32_t acc = read_payload<uint32_t>(32);
    return (double)acc / 10000;
}
double UbxNavHighPresGeo::get_horicontal_acc()
{
    uint32_t acc = read_payload<uint32_t>(28);
    return (double)acc / 10000;
}
