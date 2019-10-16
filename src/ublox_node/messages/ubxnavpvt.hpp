#pragma once
#include "ubxreceivemessage.hpp"
class UbxNavPvt : public UbxReceiveMessage
{
  private:
    /* data */
  public:
    UbxNavPvt(UbxCbInterface* cb);
    uint8_t get_gps_fix();
    uint8_t get_nav_status_flags1();
    uint8_t get_nav_status_flags2();
    double get_ground_speed();
    double get_heading();
    double get_ground_speed_acc();
    double get_heading_acc();

    static const uint8_t GPS_FIX_NO_FIX = 0;
    static const uint8_t GPS_FIX_DEAD_RECK_ONLY = 1;
    static const uint8_t GPS_FIX_2D = 2;
    static const uint8_t GPS_FIX_3D = 3;
    static const uint8_t GPS_FIX_GPS_DEAD_RECK = 4;
    static const uint8_t GPS_FIX_TIME_ONLY = 5;

    static const uint8_t FLAGS1_GPS_FIX_OK = 1;
    static const uint8_t FLAGS1_DIFF_SOL = 2;
    static const uint8_t FLAGS1_WEEK_NUMBER_VALID = 4;
    static const uint8_t FLAGS1_TOW_VALID = 8;

    static const uint8_t FIX_STATUS_DIFF_COR_AVAILABLE = 1;
    static const uint8_t FIX_STATUS_MAP_MATCHING_MASK = 192;
    static const uint8_t FIX_STATUS_MAP_MATCHING_NONE = 0;
    static const uint8_t FIX_STATUS_MAP_MATCHING_VALID_NOT_USED = 64;
    static const uint8_t FIX_STATUS_MAP_MATCHING_VALID_AND_USED = 128;
    static const uint8_t FIX_STATUS_MAP_MATCHING_ACTIVE_INCL_DR = 192;

    static const uint8_t FLAGS2_PSM_STATE_MASK = 3;
    static const uint8_t FLAGS2_PSM_STATE_ACQUISITION = 0;
    static const uint8_t FLAGS2_PSM_STATE_TRACKING = 1;
    static const uint8_t FLAGS2_PSM_STATE_PO_TRACKING = 2;
    static const uint8_t FLAGS2_PSM_STATE_INACTIVE = 3;

    static const uint8_t FLAGS2_SPOOFING_MASK = 24;
    static const uint8_t FLAGS2_SPOOFING_DEACTIVATED = 0;
    static const uint8_t FLAGS2_SPOOFING_NO_SPOOFING = 8;
    static const uint8_t FLAGS2_SPOOFING_INDICATED = 16;
    static const uint8_t FLAGS2_SPOOFING_M_INDICATED = 24;
};
