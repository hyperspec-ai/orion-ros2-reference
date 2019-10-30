#pragma once

#include "ubxreceivemessage.hpp"
#include "ubxcbinterface.hpp"
/**
* Ack Message of UBX protocol
**/
class UbxAckAck : public UbxReceiveMessage
{
  public:
    /**
    * Constructor
    *  
    * @param cb Pointer to class implementing a call back interface for this message
    */
    explicit UbxAckAck(UbxCbInterface* cb);
    /**
    * Message id for what ack was recieved
    *  
    * @return 16Bit unsigned int message id
    */
    uint16_t get_received_msg_id();
};
