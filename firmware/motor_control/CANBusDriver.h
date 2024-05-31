#ifndef CANBUS_DRIVER_H
#define CANBUS_DRIVER_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "CONBus.h"

namespace CONBus{

typedef struct{
    uint8_t registerAddress;
} CAN_readRegisterMessage;

typedef struct{
    uint8_t registerAddress;
    uint8_t length;
    uint8_t reserved_;
    uint8_t value[4];
} CAN_readRegisterResponseMessage;

typedef struct{
    uint8_t registerAddress;
    uint8_t length;
    uint8_t reserved_;
    uint8_t value[4];
} CAN_writeRegisterMessage;

typedef struct{
    uint8_t registerAddress;
    uint8_t length;
    uint8_t reserved_;
    uint8_t value[4];
} CAN_writeRegisterResponseMessage;

class CANBusDriver {
    public:
        CANBusDriver(CONBus& conbus, const uint32_t device_id);

        uint8_t readCanMessage(const uint32_t can_id, const void* buffer);
        bool isReplyReady();
        uint8_t peekReply(uint32_t& can_id, uint8_t& can_len, void* buffer);
        uint8_t popReply();

    private:
        CONBus& conbus_;
        const uint8_t device_id_;

        CAN_readRegisterMessage readRegisterMessage_;
        CAN_readRegisterResponseMessage readRegisterResponseMessage_;
        CAN_writeRegisterMessage writeRegisterMessage_;
        CAN_writeRegisterResponseMessage writeRegisterResponseMessage_;

        bool awaiting_write_response_ = false;

        void putRegisterAddressInQueue(const uint32_t register_address);
        uint8_t register_fetch_queue_[256];
        uint8_t register_fetch_queue_head_ = 0; // located at next entry to send
        uint8_t register_fetch_queue_tail_ = 0; // located *after* the last entry in the queue
};

inline CANBusDriver::CANBusDriver(CONBus& conbus, const uint32_t device_id) : conbus_(conbus), device_id_(device_id) {}

inline uint8_t CANBusDriver::readCanMessage(const uint32_t can_id, const void* buffer) {
    // CONBus read register
    if (can_id == ((uint32_t)1000 + device_id_)) {
        readRegisterMessage_ = *(CAN_readRegisterMessage*)buffer;
        if (readRegisterMessage_.registerAddress != 0xFF) {
            putRegisterAddressInQueue(readRegisterMessage_.registerAddress);
        } else {
            // Put the whole memory map into the queue
            for (int i=0; i<255; i++) {
                if (conbus_.hasRegister(i)) {
                    putRegisterAddressInQueue(i);
                }
            }
        }
    }

    // CONBus write register
    if (can_id == ((uint32_t)1200 + device_id_)) {
        awaiting_write_response_ = true;

        writeRegisterMessage_ = *(CAN_writeRegisterMessage*)buffer;
        conbus_.writeRegisterBytes(writeRegisterMessage_.registerAddress, writeRegisterMessage_.value, writeRegisterMessage_.length);

        memcpy(&writeRegisterResponseMessage_, &writeRegisterMessage_, sizeof(writeRegisterResponseMessage_));
    }

    return SUCCESS;
}

inline bool CANBusDriver::isReplyReady() {
    return (register_fetch_queue_head_ != register_fetch_queue_tail_) || awaiting_write_response_;
}

inline uint8_t CANBusDriver::peekReply(uint32_t& can_id, uint8_t& can_len, void* buffer) {
    if (register_fetch_queue_head_ != register_fetch_queue_tail_) {
        readRegisterResponseMessage_.registerAddress = register_fetch_queue_[register_fetch_queue_head_];
        conbus_.readRegisterBytes(readRegisterResponseMessage_.registerAddress, readRegisterResponseMessage_.value, readRegisterResponseMessage_.length);

        can_id = 1100 + device_id_;
        // The readRegisterResponseMessage_ is the full 7 bytes for a 4 byte buffer
        // but if we have a smaller message, we should reduce the size
        can_len = sizeof(readRegisterResponseMessage_) - (4 - readRegisterResponseMessage_.length);

        memcpy(buffer, &readRegisterResponseMessage_, sizeof(readRegisterResponseMessage_));

        return SUCCESS; // end early so we dont overwrite a read response with a write response
    }

    if (awaiting_write_response_) {
        can_id = 1300 + device_id_;
        // Same as above, we have to reduce the size appropriately.
        can_len = sizeof(writeRegisterResponseMessage_) - (4 - writeRegisterResponseMessage_.length);
        memcpy(buffer, &writeRegisterResponseMessage_, sizeof(writeRegisterResponseMessage_));
    }

    return SUCCESS;
}

inline uint8_t CANBusDriver::popReply() {
    if (register_fetch_queue_head_ != register_fetch_queue_tail_) {
        // Move head of the queue
        register_fetch_queue_head_++;

        return SUCCESS; // end early so we dont overwrite a read response with a write response
    }

    if (awaiting_write_response_) {
        awaiting_write_response_ = false;
    }

    return SUCCESS;
}

inline void CANBusDriver::putRegisterAddressInQueue(const uint32_t register_address) {
    register_fetch_queue_[register_fetch_queue_tail_] = register_address;
    register_fetch_queue_tail_++;
}


} // end CONBus namespace

#endif