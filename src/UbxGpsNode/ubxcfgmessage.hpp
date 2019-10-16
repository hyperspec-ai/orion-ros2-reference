#pragma once
#include "ubxmessage.hpp"
/**
 * Config message interface
 **/
class UbxCfgMessage : public UbxMessage
{
   protected:
    /**
     * Constructor
     *
     * @param uint8_t class_id class type of mesage (ubx protocol specific)
     * @param uint8_t msg_id message id of mesage (ubx protocol specific)
     */
    UbxCfgMessage(UbxMessage::message_class_t class_id, UbxMessage::message_id_t msg_id);
    /**
     * Function to set the layer config data should be stored in a generic config message
     *
     * @param const uint8_t layer Possible Values: LAYER_RAM, LAYER_BBR, LAYER_FLASH
     */
    void set_layer(const uint8_t layer);
    static const uint8_t LAYER_RAM = 1;
    static const uint8_t LAYER_BBR = 2;
    static const uint8_t LAYER_FLASH = 4;
};
