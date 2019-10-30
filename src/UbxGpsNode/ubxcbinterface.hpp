#pragma once

#include <cstdint>
#include "ubxmessage.hpp"
/**
 * Interface to be implement call back for received messages
 **/
class UbxCbInterface
{
  public:
    /**
     * Call back function definition
     *
     * @param id of message received (combnation of ubx message id and class)
     * @param msg pointer to received message
     */
    virtual void message_received(UbxMessage::Identifier id, const UbxMessage* msg) = 0;
};
