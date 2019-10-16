#pragma once

#include "ubxreceivemessage.hpp"
#include "ubxcbinterface.hpp"

/**
* nak Message of UBX protocol
**/
class UbxAckNak : public UbxReceiveMessage
{

  public:
    /**
    * Constructor
    *  
    * @param Pointer to class implementing a call back interface for this message
    */
    explicit UbxAckNak(UbxCbInterface* cb);
    /**
    * Message id for what ack was recieved
    *  
    * @return 16Bit unsigned int message id
    */
    uint16_t get_received_msg_id();
};
