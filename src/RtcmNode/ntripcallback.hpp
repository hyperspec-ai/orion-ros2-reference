#pragma once
#include <cstdint>
#include <vector>
namespace ntrip {
class NtripCallback
{
  public:
    /**
     * Callbak for received data
     * @param data data holds one transmission, can include multiple RTCM
     * sentences
     **/
    virtual void data_received(std::vector<uint8_t> data) = 0;
};
} // namespace ntrip
