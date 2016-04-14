/**
 * @file
 * @brief Dynamixel Device Library
 */
#ifndef SerialDXL_h
#define SerialDXL_h
//------------------------------------------------------------------------------
/**
 * Debug macros
 */
#ifdef DEBUG_SERIAL_DXL
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif
//------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <string.h>
#include "mmap.h"
#include <Arduino.h>
//------------------------------------------------------------------------------
/**
 * @class DeviceDXL
 * @brief Virtual class for device with Dynamixel protocol.
 */



class DeviceDXL {
  public:
    DeviceDXL(uint8_t id, uint8_t N):
    id_(id),
    mmap_(N)
    {}
    virtual void initRAM() {}

    virtual void initEEPROM() {}

    virtual inline __attribute__((always_inline))
    void setTX() {}

    virtual inline __attribute__((always_inline))
    void setRX() {}

    // ID
    uint8_t id_;

    // Baudrate
    uint8_t baudrate_;

    // Return dalay time
    uint8_t returnDelay_;

    // Memory mapping
    MMap::MMap mmap_;
};
//------------------------------------------------------------------------------
/**
 * @class SerialDXL
 * @brief Serial port wrapper for DXL communication protocol.
 */

#define SERIALDXL_MSG_LENGTH 64
/** Cause error message for RX buffer bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badMsgBufLength(void)
  __attribute__((error("Message buffer length too large or zero")));


class SerialDXL
{
  public:
    SerialDXL():
      msgState_(0U),
      msgParamIdx_(2U),
      msgLen_(0U),
      msgFinish_(0U),
      msgChecksum_(0U),
      error_(0U)
    {
    }

    /**
     * @brief Initialize SerialDXL. Set baudrate and target device.
     * @details 
     * 
     * @param baud Baudrate using Dynamixel format.
     * @param port Serial port (Stream class).
     * @param device Target device.
     */
    void init(uint32_t baud, HardwareSerial *port, DeviceDXL *device)
    {
      // Set DeviceDXL
      device_ = device;
      // Set serial port
      port_ = port;
      // Set baudrate using Dynamixel relation
      port_->begin(baud);
    }

    /**
     * @brief Process data from Serial 
     * @details Add data to buffers
     * 
     * @param data Data from Serial port.
     */
    void process(uint8_t data)
    {
      switch(msgState_)
      {
        case 0: // 0xFF
          msgState_ = data == 0xFF ? 1 : 0;
          break;
          
        case 1: // 0XFF
          msgState_ = data == 0xFF ? 2 : 1;
          break;
          
        case 2: // ID
          // Check error
          msgState_ = data == device_->id_ ? 3 : 0;
          break;
          
        case 3: // Length
          msgLen_ = data;
          // Checksum
          msgChecksum_ += device_->id_ + data;
          // Save length in the RX message buffer
          rxMsgBuf_[0] = data;
          msgState_ = 4;
          break;

        case 4: // Instruction
          // Save instruction in the RX message buffer
          rxMsgBuf_[1] = data;
          // Checksum
          msgChecksum_ += data;
          // Check for short message
          msgState_ = msgLen_ <= 2 ? 6 : 5;
          break;
        
        case 5: // Parameters
          rxMsgBuf_[msgParamIdx_++] = data;
          // Checksum
          msgChecksum_ += data;
          // Check message length
          msgState_ = msgParamIdx_ >= msgLen_ ? 6 : msgState_;
          break;

        case 6: // Checksum
          rxMsgBuf_[msgParamIdx_] = data;
          // Check
          msgFinish_ = ((uint8_t)(~msgChecksum_))==data;
          // Reset states
          msgState_ = 0;
          msgParamIdx_ = 2;
          msgLen_ = 0;
          msgChecksum_ = 0;
          break;
      }

      // Process message
      if (msgFinish_)
      {
        uint8_t i;
        switch(rxMsgBuf_[1])
        {
          case 1: // Ping
            txMsgBuf_[0] = 0xff;
            txMsgBuf_[1] = 0xff;
            txMsgBuf_[2] = device_->id_; //ID 
            txMsgBuf_[3] = 2; // Length
            txMsgBuf_[4] = 0; // Error
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            _delay_us(160);
            
            device_->setTX();
            // Send
            port_->write(txMsgBuf_,6);
            port_->flush(); // Wait to complete

            device_->setRX();
            msgFinish_ = 0;
            break;

          case 2: // Read data
            txMsgBuf_[0] = 0xff;
            txMsgBuf_[1] = 0xff;
            txMsgBuf_[2] = device_->id_; //ID 
            txMsgBuf_[3] = 3; // Length
            txMsgBuf_[4] = 0; // Error
            // @TODO
            txMsgBuf_[5] = device_->mmap_.get(rxMsgBuf_[2]);
            
            txMsgBuf_[6] = ~(txMsgBuf_[2]+txMsgBuf_[3]+txMsgBuf_[4]+txMsgBuf_[5]);
            // Status return delay
            _delay_us(160);

            device_->setTX();
            // Send
            port_->write(txMsgBuf_, 7);
            port_->flush(); // Wait to complete

            device_->setRX();
            msgFinish_ = 0;
            break;

          case 3: // Write data
            txMsgBuf_[0] = 0xff;
            txMsgBuf_[1] = 0xff;
            txMsgBuf_[2] = device_->id_; //ID 
            txMsgBuf_[3] = 2; // Length

            // @TODO
            // Only one byte
            device_->mmap_.set(rxMsgBuf_[2],rxMsgBuf_[3]);

            txMsgBuf_[4] = 0; // Error
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            _delay_us(160);
            
            device_->setTX();
            // Send
            port_->write(txMsgBuf_, 6);
            port_->flush(); // Wait to complete
            
            device_->setRX();
            msgFinish_ = 0;
            break;

        }
      }
    }

    uint8_t* getMsg()
    {
      return (msgFinish_) ? rxMsgBuf_ : NULL;
    }

    void reset()
    {
      msgFinish_ = 0;
    }


  private:
    // DeviceDXL
    DeviceDXL *device_;
    // Serial port
    HardwareSerial *port_;
    // Mesage reception state
    uint8_t msgState_;
    uint8_t msgParamIdx_;
    uint8_t msgLen_;
    uint8_t msgFinish_;
    uint16_t msgChecksum_;

    // Error state
    uint8_t error_;

    // Receive message buffer
    uint8_t rxMsgBuf_[SERIALDXL_MSG_LENGTH];
    // Send message buffer
    uint8_t txMsgBuf_[SERIALDXL_MSG_LENGTH];
};


#endif