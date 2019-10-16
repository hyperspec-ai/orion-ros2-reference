#pragma once
#include "mqtt/async_client.h"

class MqttCallback
{
   public:
    /**
     * Callbac called when a subscried message was received
     *
     * @param mqtt::const_message_ptr msg message received
     **/
    virtual void message_recieved(mqtt::const_message_ptr msg) = 0;
};
