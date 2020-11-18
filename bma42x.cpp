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
#include "mbed.h"
#include "bma42x.h"

/**
 *@brief Constructor
 *@param I2C *i2c reference to i2c
 *@param uint8_t deviceAddress slaveaddress
 */
BMA42X::BMA42X(I2C *i2c, uint8_t deviceAddress) : _i2c(i2c) {
  
  _writeOpcode = deviceAddress & 0xFE; // low order bit = 0 for write
  _readOpcode  = deviceAddress | 0x01; // low order bit = 1 for read  
  
  initialize(); 
}

/** @brief High level Init, most settings remain at Power-On reset value
 */
void BMA42X::initialize() {
}

/** @brief Write command that has no parameters
*/ 
void BMA42X::_sendCommand(uint8_t command) {
//  I2Cdev::writeByte(m_devAddr, COMMAND_MODE, command);

#if (I2C_OPTIMIZE == 0)
  char databytes[2];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  _i2c->write(_writeOpcode, databytes, 2);    // Write command   
#else  

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   

  _i2c->stop();  
#endif
}

/** @brief Write command that has one parameter
*/ 
void BMA42X::_sendCommand(uint8_t command, uint8_t param1) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[4];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  _i2c->write(_writeOpcode, databytes, 4);    // Write command   
#else  

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   

  _i2c->stop();
#endif  
}

/** @brief Write command that has two parameters
*/ 
void BMA42X::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[6];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  _i2c->write(_writeOpcode, databytes, 6);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   

  _i2c->stop();
 #endif 
}

/** @brief Write command that has five parameters
*/ 
void BMA42X::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                            uint8_t param3, uint8_t param4,
                                            uint8_t param5) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[12];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  databytes[6] = COMMAND_MODE;
  databytes[7] = param3; 
  databytes[8] = COMMAND_MODE;
  databytes[9] = param4; 
  databytes[10] = COMMAND_MODE;
  databytes[11] = param5;       
  _i2c->write(_writeOpcode, databytes, 12);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param3);        // Write Param3   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param4);        // Write Param4   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param5);        // Write Param5   

  _i2c->stop();
#endif  
}


/** @brief Write command that has six parameters
*/ 
void BMA42X::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                            uint8_t param3, uint8_t param4,
                                            uint8_t param5, uint8_t param6) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[14];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  databytes[6] = COMMAND_MODE;
  databytes[7] = param3; 
  databytes[8] = COMMAND_MODE;
  databytes[9] = param4; 
  databytes[10] = COMMAND_MODE;
  databytes[11] = param5;   
  databytes[12] = COMMAND_MODE;
  databytes[13] = param6;       
  _i2c->write(_writeOpcode, databytes, 14);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param3);        // Write Param3   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param4);        // Write Param4   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param5);        // Write Param5   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param6);        // Write Param6   

  _i2c->stop();
#endif  
}


#if(0)
/** @brief Write command that has multiple parameters
*/ 
void SSD1308::_sendCommands(uint8_t len, uint8_t* commands) {

//  I2Cdev::writeBytes(m_devAddr, COMMAND_MODE, len, commands);
//  Note this original code is not correct, continuationbit is set, 
//  so COMMAND_MODE must be repeated before each databyte that serves as parameter!

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  for (int i=0; i<len ; i++) {
    _i2c->write(COMMAND_MODE);      
    _i2c->write(commands[i]);  // Write Commands   
  }
  _i2c->stop();
  
}
#endif

/** @brief Write databyte to display
 *  @brief Start at current cursor location
 *  @param uint8_t data databyte to write
*/
void BMA42X::_sendData(uint8_t data){

#if (I2C_OPTIMIZE == 0)
//I2C Blockwrite versions dont seem to work ?
//That may be related to fact that the SSD1308/SSD1306 does NOT return an acknowledge: blockwrite may abort the operation
//Noted for mbed lib v63 on 20/7/13 
  char databytes[2];
    
  databytes[0] = DATA_MODE;
  databytes[1] = data;    
  _i2c->write(_writeOpcode, databytes, 2);    // Write Data   

#else
  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  _i2c->write(data); 
  _i2c->stop();  
#endif

}

/** @brief Write len bytes from buffer data to display, 
 *  @brief Start at current cursor location
 *  @param uint8_t len number of bytes to write 
 *  @param uint8_t* data pointer to data
*/
void BMA42X::_sendData(uint8_t len, uint8_t* data) {
//  I2Cdev::writeBytes(m_devAddr, DATA_MODE, len, data);
#if (I2C_OPTIMIZE == 0)
  for (int i=0; i<len ; i++) {
    _sendData(data[i]);  // Write Data   
  }
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  for (int i=0; i<len ; i++) {
    _i2c->write(data[i]);  // Write Data   
  }
  _i2c->stop();
#endif 
}

/** @brief Low level Init
 *  @brief Init the configuration registers in accordance with the datasheet
 */
void SSD1308::_init() {

  // _sendCommand(SET_DISPLAY_POWER_OFF);      // 0xAE
}

