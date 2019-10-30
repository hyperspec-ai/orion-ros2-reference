#pragma once
#include <cstdio>
#include "ubxcbinterface.hpp"
#include "ubxmessage.hpp"
/**
 * Receive Message interface definition
 **/
class UbxReceiveMessage : public UbxMessage
{
  public:
    /**
     * Constructor
     *
     * @param class_id class type of mesage (ubx protocol specific)
     * @param msg_id message id of mesage (ubx protocol specific)
     * @param cb Pointer to class implementing a call back interface for this
     * message
     */
    UbxReceiveMessage(UbxMessage::message_class_t class_id,
                      UbxMessage::message_id_t msg_id,
                      UbxCbInterface* cb);
    /**
     * Set Message Content and inform registered call back if not NULL
     *
     * @param buffer Pointer to message buffer to read out
     * @param length size of buffer
     */
    void read_message_from(const uint8_t* buffer, const uint16_t length);

  private:
    /// Pointer to registered callback interface
    UbxCbInterface* m_cb = NULL;
};
