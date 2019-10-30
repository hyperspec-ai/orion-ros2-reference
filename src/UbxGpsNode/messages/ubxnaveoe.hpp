#pragma once

#include "ubxreceivemessage.hpp"
#include "ubxcbinterface.hpp"
/**
* End of epoch Message of UBX protocol, refelcts the end of all messages for one postion resolution
**/
class UbxNavEoe : public UbxReceiveMessage
{

  public:
    /**
    * Constructor
    *  
    * @param cb Pointer to class implementing a call back interface for this message
    */
    explicit UbxNavEoe(UbxCbInterface* cb);
    /**
    * GPS Resolution epoch
    *  
    * @return 32Bit timestamp reflecting the epoch of the position resolution
    */
    uint32_t get_itow();
};
