#ifndef INCLUDE_SENSORS_TELEMETRY_UbxNavRateCfg_MESSAGE_HPP_
#define INCLUDE_SENSORS_TELEMETRY_UbxNavRateCfg_MESSAGE_HPP_
#include "ubxcfgmessage.hpp"
class UbxNavRateCfg : public UbxCfgMessage
{
  public:
    UbxNavRateCfg();
    void set_meas_rate_ms(uint16_t rate);
    void set_rate_nav(uint16_t rate);
    void set_rate_time_ref(uint8_t mode);
    void set_dyn_model(uint8_t mode);
    static const uint8_t TIMEREF_UTC = 0;
    static const uint8_t TIMEREF_GPS = 1;
    static const uint8_t TIMEREF_GLO = 2;
    static const uint8_t TIMEREF_BDS = 3;
    static const uint8_t TIMEREF_GAL = 4;
    static const uint8_t DYNMODE_PORTABLE = 0;
    static const uint8_t DYNMODE_STATIONARY = 2;
    static const uint8_t DYNMODE_PEDESTRIAN = 3;
    static const uint8_t DYNMODE_AUTOMOTIVE = 4;
    static const uint8_t DYNMODE_SEA = 5;
    static const uint8_t DYNMODE_AIR1 = 6;
    static const uint8_t DYNMODE_AIR2 = 7;
    static const uint8_t DYNMODE_AIR4 = 8;
};
#endif /*INCLUDE_SENSORS_TELEMETRY_UbxNavRateCfg_MESSAGE_HPP_*/