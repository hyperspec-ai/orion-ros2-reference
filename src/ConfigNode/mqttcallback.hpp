#pragma once
#include "mqtt/async_client.h"
/**
 * @brief Call nack interface for received mqtt messages
 * 
 */
class MqttCallback
{
  public:
    /**
     * Callbac called when a subscried message was received
     *
     * @param  msg message received
     **/
    virtual void message_recieved(mqtt::const_message_ptr msg) = 0;
};
