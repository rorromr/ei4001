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
#include <avr/sleep.h>
#include "mmap.h"
#include <Arduino.h>
//------------------------------------------------------------------------------
/**
 * @class DeviceDXL
 * @brief Virtual class for device with Dynamixel protocol.
 */

class DeviceDXL {
  public:
    DeviceDXL(uint16_t model, uint8_t firmware, uint8_t N):
      model_l_(0, 0, MMap::Access::R, 0, 255, (model >> 8*0) & 0xFF), // Low bit
      model_h_(1, 1, MMap::Access::R, 0, 255, (model >> 8*1) & 0xFF), // High bit
      firmware_(2, 2, MMap::Access::R, 0, 255, firmware),
      id_(3, 3, MMap::Access::RW, 1, 254, 1U),
      baudrate_(4, 4, MMap::Access::RW, 1, 254, 34U),
      return_delay_(5, 5, MMap::Access::RW, 1, 254, 160U),
      mmap_(N+6)
    {
    }

    virtual inline bool onReset() = 0;

    virtual inline void setTX() = 0;

    virtual inline void setRX() = 0;

    void init(uint8_t *msgBuffer, VariablePtr *varList)
    {
      mmap_.init(msgBuffer, varList);
      mmap_.registerVariable(&model_l_);
      mmap_.registerVariable(&model_h_);
      mmap_.registerVariable(&firmware_);
      mmap_.registerVariable(&id_);
      mmap_.registerVariable(&baudrate_);
      mmap_.registerVariable(&return_delay_);
      mmap_.load(); // Load values from EEPROM
    }

    void reset()
    {
      if (onReset())
      {
        mmap_.reset(); // Set default values and save EEPROM
      }
    }

    // Model
    MMap::UInt8NV model_l_; // Low bit
    MMap::UInt8NV model_h_; // High bit

    // Firmware version
    MMap::UInt8NV firmware_;

    // ID
    MMap::UInt8NV id_;

    // Baudrate
    MMap::UInt8NV baudrate_;

    // Return dalay time
    MMap::UInt8NV return_delay_;

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
      // TODO Set baudrate from device info
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
          msgState_ = data == device_->id_.data ? 3 : 0;
          break;
          
        case 3: // Length
          msgLen_ = data;
          // Checksum
          msgChecksum_ += device_->id_.data + data;
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
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 2; // Length
            txMsgBuf_[4] = 0; // Error
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            _delay_us(device_->return_delay_.data);
            
            device_->setTX();
            // Send
            port_->write(txMsgBuf_,6);
            port_->flush(); // Wait to complete

            device_->setRX();
            msgFinish_ = 0;
            break;

          case 2: // Read data
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 3; // Length
            txMsgBuf_[4] = 0; // Error
            // TODO Add more than one value
            txMsgBuf_[5] = device_->mmap_.get(rxMsgBuf_[2]);
            
            txMsgBuf_[6] = ~(txMsgBuf_[2]+txMsgBuf_[3]+txMsgBuf_[4]+txMsgBuf_[5]);
            // Status return delay
            _delay_us(device_->return_delay_.data);

            device_->setTX();
            // Send
            port_->write(txMsgBuf_, 7);
            port_->flush(); // Wait to complete

            device_->setRX();
            msgFinish_ = 0;
            break;

          case 3: // Write data
            txMsgBuf_[0] = 0xFF;
            txMsgBuf_[1] = 0xFF;
            txMsgBuf_[2] = device_->id_.data; //ID 
            txMsgBuf_[3] = 2; // Length

            // TODO Add more than one value
            device_->mmap_.set(rxMsgBuf_[2],rxMsgBuf_[3]);

            txMsgBuf_[4] = 0; // Error
            txMsgBuf_[5] = ~(txMsgBuf_[2]+txMsgBuf_[3]);
            // Status return delay
            _delay_us(device_->return_delay_.data);
            
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