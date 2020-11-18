/**  @file bma42x I2C device class file
/* 
================================================================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2020 Mark Winney (mbed port)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
================================================================================
*/

#ifndef BMA24X_H
#define BMA24X_H

// This is the I2C address (8 bit)
#define BMA42X_SA0                 0x78

// Command and Datamode 
#define COMMAND_MODE               0x80 // continuation bit is set!
#define DATA_MODE                  0x40

/** Class to control an BMA42X (X=1,3) sensor
 *
 * Example:
 * @code
 * #include "mbed.h"
 *
 * @endcode
 */
class BMA42X : public Stream {
public:

    /**
     *@brief Constructor
     *@param I2C &i2c reference to i2c,
     *@param uint8_t deviceAddress slaveaddress (8bit to use for the controller (0x78 by default))
    */    
    BMA42X(I2C *i2c, uint8_t address = BMA42X_SA0);

    // High Level methods

    /** @brief High level Init, most settings remain at Power-On reset value
     */
    void initialize();

private:

    // Low Level methods

    /** @brief Write command that has no parameters
    */    
    void _sendCommand(uint8_t command); 

    /** @brief Write command that has one parameter
    */    
    void _sendCommand(uint8_t command, uint8_t param1);

    /** @brief Write command that has two parameters
    */ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2);              
    //    void sendCommands(uint8_t len, uint8_t* buf);

    /** @brief Write command that has five parameters
    */ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                    uint8_t param3, uint8_t param4,
                                    uint8_t param5);

    /** @brief Write command that has six parameters
    */ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                    uint8_t param3, uint8_t param4,
                                    uint8_t param5, uint8_t param6);
        
    /** @brief Write databyte to display
     *  @brief Start at current cursor location
     *  @param uint8_t data databyte to write
    */  
    void _sendData(uint8_t data);

    /** @brief Write len bytes from buffer data to display, 
     *  @brief Start at current cursor location
     *  @param uint8_t len number of bytes to write 
     *  @param uint8_t* data pointer to data
    */   
    void _sendData(uint8_t len, uint8_t* data);

    /** @brief Low level Init
     *  @brief Init the configuration registers in accordance with the datasheet
     */   
    void _init();

    I2C *_i2c;             // I2C bus reference
    uint8_t _readOpcode;   // contains the I2C address of the device
    uint8_t _writeOpcode;  // contains the I2C address of the device
};

#endif
