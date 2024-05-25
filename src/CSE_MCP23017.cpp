
//============================================================================================//
// Includes

#include "CSE_MCP23017.h"

//============================================================================================//
// Globals

// CSE_MCP23017 library supports managing multiple IO expander objects on the same/different bus.
// Interrupts are fully supported for all IO expanders simultaneously. To makle this process
// easier, the library maintains a list of all IO expanders on the bus, by saving pointers to
// the IO expander objects in a global array `ioeList` and keeps track of the number of IO expanders
// using the `ioeCount` variable.
CSE_MCP23017* ioeList [MCP23017_MAX_OBJECT] = {0};  // Pointers to all IO expander objects
uint8_t ioeCount = 0;  // IO exapander object count

//============================================================================================//
// Templates

template <int index>  // Receives the index to the global arrays
void callback() {
  if (ioeList [index] != nullptr) {  // If the pointer is not zero
    if (!ioeList [index]->interruptPending()) { // If an interupt is not already being served
      debugPort.print (F("callback(): Callback invoked at "));
      debugPort.println (index);
      debugPort.print (F("callback(): Object is at 0x"));
      debugPort.println (uint32_t (ioeList [index]), HEX);
      debugPort.println();

      // Activate the interrupt so that next time the ISR dispatcher is called
      // the ISR will be executed.
      ioeList [index]->interruptActive = true;
      return;
    }
    else {
      debugPort.print (F("callback(): \n--\nInterrupt pending at "));
      debugPort.println (index);
      debugPort.println (F("--\n"));
      return;
    }
  }
  else {
    debugPort.print (F("callback(): Callback error at "));
    debugPort.println (index);
    debugPort.println();
  }
}

// Array of callbacks for the host MCU
hostCallback_t hostCallbackList [MCP23017_MAX_OBJECT] = {
  callback <0>,
  callback <1>,
  callback <2>,
  callback <3>,
  callback <4>,
  callback <5>
};


//============================================================================================//

CSE_MCP23017:: CSE_MCP23017 (uint8_t rstPin, uint8_t address = MCP23017_ADDRESS) {
  deviceAddress = address;
  resetPin = rstPin;
  
  regBank [MCP23017_REG_IODIRA] = 0xFF; // Reset values
  regBank [MCP23017_REG_IODIRB] = 0xFF;

  intPin = -1;
  intPinState = -1;
  intPinCapState = -1;
  lastIntPin = -1;

  deviceReadError = false;
  deviceWriteError = false;
  interruptActive = false;
  stateReverted = true;
  
  // Assign the callback for this object
  callback = hostCallbackList [ioeCount];
  ioeIndex = ioeCount;

  // Save the obj ptr to the global list, and update obj count
  ioeList [ioeCount++] = this;
}

//--------------------------------------------------------------------------------------------//
//destructor

CSE_MCP23017:: ~CSE_MCP23017() {
}

//============================================================================================//
/**
 * @brief Resets the IOE though the dedicated reset pin. This pin should be connected the
 * host microcontroller for controlling it.
 * 
 * The library maintains a shadow copy of the IO exander register bank. This allows the user
 * to perform multiple operations on the registers and update all at once with an `update()` call.
 * This removes the extra overhead of several I2C transmissions and delays.
 * 
 */
void CSE_MCP23017:: reset() {
  bankMode = PAIR;
  addressMode = 0;

  // Clear the shadow copy of the register bank by writing all 0s.
  for (int i = 0; i <= MCP23017_REGADDR_MAX; i++) {
    regBank [i] = 0;
  }
  
  // To make the reset state of the registers as per the device state,
  // we must also set the IODIRA and IODIRB registers to the following according
  // to the datasheet.
  regBank [MCP23017_REG_IODIRA] = 0xFF; // Reset values
  regBank [MCP23017_REG_IODIRB] = 0xFF;
  
  // Set the pin mode to output using the global Arduino-API.
  ::pinMode (resetPin, OUTPUT);
  
  // Assert the reset sequnce. All these functions are from the Arduino framework.
  ::digitalWrite (resetPin, LOW);
  ::delay (10);
  ::digitalWrite (resetPin, HIGH);
}

//============================================================================================//
//check if the device is present
//hardware resets the device
/**
 * @brief Checks the presence of the MCP23017 on the I2C bus by reading the ACK response.
 * 
 * @return uint8_t The response from the Wire library.
 */
uint8_t CSE_MCP23017:: begin() {
  Wire.beginTransmission (deviceAddress);
  uint8_t response = Wire.endTransmission(); // Read ACK
  
  if (response == 0) {
    debugPort.println (F("begin(): MCP23017 is found on the bus."));
  }
  else {
    debugPort.println (F("begin(): MCP23017 is not found on the bus."));
  }

  reset();
  return response;
}

//============================================================================================//
/**
 * @brief Directly writes a sequence of bytes to the IOE. These values are not saved to the
 * local register bank. You must call `readAll()` to update the local register bank.
 * If translateAddress is false, absolute register address will be used.
 * 
 * @param regAddress Starting register address.
 * @param buffer A pointer to a byte buffer.
 * @param bufferOffset A position offset in the buffer where the reading will begin from.
 * @param length The number of bytes to write.
 * @param translateAddress Whether to translate the register address or not.
 * @return uint8_t Response from the Wire library.
 */
uint8_t CSE_MCP23017:: write (uint8_t regAddress, uint8_t *buffer, uint8_t bufferOffset, uint8_t length, bool translateAddress) {
  Wire.beginTransmission (deviceAddress);
  Wire.write (regAddress);
  
  for (uint8_t i = bufferOffset; i < (bufferOffset + length); i++) {
    Wire.write (buffer [i]);
  }

  uint8_t response = Wire.endTransmission();

  if (response != MCP23017_RESP_OK) {
    writeError (true);
  }
  return response;
}

//--------------------------------------------------------------------------------------------//
/**
 * @brief Directly writes a single byte to the IOE. The data is not saved in the local register bank.
 * You must call `readAll()` to update the local register bank.
 * Bank or Sequential mode doesn't matter when writing a single byte.
 * 
 * @param regAddress Register address.
 * @param data Data to be written. 
 * @param translateAddress Whether to translate the register address or not.
 * @return uint8_t Response from the Wire library in case of communication error.
 */
uint8_t CSE_MCP23017:: write (uint8_t regAddress, uint8_t data, bool translateAddress) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  // Check if the address is in range
    Wire.beginTransmission (deviceAddress); // Write device address

    if (translateAddress) {  // If bankmode = 1 (group)
      Wire.write (TRANSLATE (regAddress));  // Translate the address for both port A and B
    }
    else {
      Wire.write (regAddress);
    }

    Wire.write (data);
    uint8_t response = Wire.endTransmission();

    if (response != MCP23017_RESP_OK) {
      writeError (true);
    }

    return response;
  }

  writeError (true);
  return MCP23017_ERROR_OOR;  // Address out of range
}

//============================================================================================//
/**
 * @brief Updates all IOE registers with the local register bank values.
 * 
 * TODO: Rename this function to `writeAll()`.
 * 
 * @param translateAddress Whether to translate the register address or not.
 * @return uint8_t Response from the Wire library.
 */
uint8_t CSE_MCP23017:: write (bool translateAddress) {
  Wire.beginTransmission (deviceAddress);
  Wire.write (TRANSLATE (MCP23017_REG_IODIRA));
  
  for (int i = 0; i <= MCP23017_REGADDR_MAX; i++) {
    if (translateAddress) {
      // In bank mode, or when address translation is needed,
      // we need to fetch from the correct locations in the local reg bank
      // writes to device sequentially, addressMode should be 0
      Wire.write (regBank [TRANSLATE (i)]);
    }
    else {
      //otherwise read values in sequential order
      Wire.write (regBank [i]);
    }
  }

  uint8_t response = Wire.endTransmission();
  
  if (response != MCP23017_RESP_OK) {
    writeError (true);
  }

  return response;
}

//============================================================================================//
/**
 * @brief Read a single byte from the device register. Returns the value read from the register.
 * In case of error, `0xFF` is returned. You must check for `readError()` to know if an error occurred.
 * 
 * TODO : Add functions to read multiple values in single go
 * 
 * @param regAddress Address of the register.
 * @param translateAddress Whether to translate the register address or not.
 * @return uint8_t Value read from the register. 0xFF in case of error.
 */
uint8_t CSE_MCP23017:: read (uint8_t regAddress, bool translateAddress) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  // Check if address is in range
    Wire.beginTransmission (deviceAddress);
    if (translateAddress) {  // If bankmode = 1 (group)
      Wire.write (TRANSLATE (regAddress));  // Translate address for both port A and B
    }
    else {
      Wire.write (regAddress);
    }
    Wire.endTransmission();

    Wire.requestFrom (deviceAddress, 1, true); // Address, Quantity and Bus release

    if (Wire.available() == 1) {
      return uint8_t (Wire.read());
    }
    
    debugPort.print (F("read(): Device 0x"));
    debugPort.print (deviceAddress, HEX);
    debugPort.println (F(" not responding"));
    readError (true);
    return 0xFF;
  }

  debugPort.println (F("read(): MCP23017 Error - Value out of range"));
  return MCP23017_ERROR_OOR;  // Address out of range
}

//============================================================================================//
/**
 * @brief Returns the last read error state of the I2C read operation.
 * The erorr state is reset after this function is called. Successfull read operations will not
 * reset a previously set error state. The user must call this function for resetting.
 * 
 * @return true There was a read error since the last time this function was called.
 * @return false No read error since the last time this function was called.
 */
bool CSE_MCP23017:: readError() {
  if (deviceReadError) {
    deviceReadError = false;  // Reset the error state
    return true;
  }
  return false;
}

//============================================================================================//
/**
 * @brief Sets the error state for I2C read operations. The user must always pass the state
 * in order to avoid ambiguity with the overloaded function.
 * 
 * @param err The error state. Can be `true` or `false`.
 */
void CSE_MCP23017:: readError (bool err) {
  deviceReadError = err;
}

//============================================================================================//
/**
 * @brief Returns the last write error state of the I2C write operation.
 * The error state is reset after this function is called. Successful write operations will not
 * reset a previously set error state. The user must call this function for resetting.
 * 
 * @return true There was a write error since the last time this function was called.
 * @return false No write error since the last time this function was called.
 */
bool CSE_MCP23017:: writeError() {
  if (deviceWriteError) {
    deviceWriteError = false; // Reset the error state
    return true;
  }
  return false;
}

//============================================================================================//
//sets the error state for the write operations
//should pass the state to avoid ambiguity
/**
 * @brief Sets the error state for I2C write operations. The user must always pass the state
 * in order to avoid ambiguity with the overloaded function.
 * 
 * @param err The error state. Can be `true` or `false`.
 */
void CSE_MCP23017:: writeError (bool err) {
  deviceWriteError = err;
}

//============================================================================================//
/**
 * @brief Read all registers from the device and store them in the local register bank.
 * 
 * @param translateAddress Whether to translate the register address or not.
 * @return uint8_t Returns `0` on success.
 */
uint8_t CSE_MCP23017:: readAll (bool translateAddress) {
  Wire.beginTransmission (deviceAddress); // Write device address
  Wire.write (MCP23017_REG_IODIRA);
  Wire.endTransmission();

  Wire.requestFrom (deviceAddress, 22);

  for (int i = 0; i <= MCP23017_REGADDR_MAX; i++) {
    uint8_t incomingByte = Wire.read();
    if (translateAddress) {
      regBank [TRANSLATE (i)] = incomingByte;
    }
    else {
      regBank [i] = incomingByte;
    }
  }

  return 0;
}

//============================================================================================//
/**
 * @brief Updates the local register bank from a buffer. The values are not written to the device
 * until you call 'writeAll()'. Use `write()` functions for fast writing to the device without
 * saving the data to the local register bank.
 * 
 * @param regAddress The starting register address.
 * @param buffer The source buffer.
 * @param bufferOffset A location offset in the buffer from the 0th byte.
 * @param length The number of bytes to read/update.
 * @return uint8_t The error code.
 */
uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t *buffer, uint8_t bufferOffset, uint8_t length) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  // Check if address is in range
    for (uint8_t i = bufferOffset; i < (bufferOffset + length); i++) {
      regBank [regAddress++] = buffer [i];
    }
    return MCP23017_RESP_OK;
  }
  return MCP23017_ERROR_OOR;
}

//--------------------------------------------------------------------------------------------//
/**
 * @brief Updates the local register bank with two bytres of data. The values are not
 * written to the device until you call 'writeAll()'. Use `write()` functions for fast writing
 * to the device without saving the data to the local register bank.
 * 
 * @param regAddress The starting register address.
 * @param byteOne First byte.
 * @param byteTwo Second byte.
 * @return uint8_t The error code.
 */
uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t byteOne, uint8_t byteTwo) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    regBank [regAddress] = byteOne;
    regBank [regAddress + 1] = byteTwo;
    return MCP23017_RESP_OK;
  }

  return MCP23017_ERROR_OOR;
}

//--------------------------------------------------------------------------------------------//
/**
 * @brief Updates the local register bank with one byte of data. The values are not written
 * to the device until you call 'writeAll()'. Use `write()` functions for fast writing to the
 * device without saving the data to the local register bank.
 * 
 * @param regAddress The starting register address.
 * @param byteOne First byte.
 * @return uint8_t The error code.
 */
uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t byteOne) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    regBank [regAddress] = byteOne;
    return MCP23017_RESP_OK;
  }
  
  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Sets the GPIO direction of both ports A and B. The pin numbers can be from 0-15.
 * The first 8 pins belongs to port A (0-7) and the rest to port B (8-15).
 * Modes can be `INPUT` (`0`), `OUTPUT` (`1`) or `INPUT_PULLUP` (`2`).
 * Though for the IO expander, `1` means INPUT and `0` means OUTPUT, the dissimilarity is to keep
 * the library APIs compatible with the original Arduino-style GPIO APIs.
 * 
 * The function first writes to the device and if the operation is successful, the values
 * are stored in the local register bank.
 * 
 * @param pin The GPIO pin. Can be from 0-15.
 * @param mode Pin mode. Can be `INPUT` (`0`), `OUTPUT` (`1`) or `INPUT_PULLUP` (`2`).
 * @return uint8_t 
 */
uint8_t CSE_MCP23017:: pinMode (uint8_t pin, uint8_t mode) {
  if ((pin < MCP23017_PINCOUNT) && (mode < MCP23017_PINMODES)) {  // Check if values are in range
    // First, save the new register value to a temp variable,
    // so that if the write fails for any reason, we can retain the
    // original value in the local register bank.
    
    uint8_t pinModeByte = 0;
    uint8_t pullupModeByte = 0;
    uint8_t response_1, response_2 = 0;

    debugPort.print (F("pinMode(): Setting pin mode at "));
    debugPort.println (pin);
    debugPort.println (F("pinMode(): Reading registers"));
    
    // Read the values from the device and store it at local register bank.
    // Bank Mode can tell if an address translation is needed or not.
    regBank [MCP23017_REG_IODIRA] = read (MCP23017_REG_IODIRA, false);
    regBank [MCP23017_REG_IODIRB] = read (MCP23017_REG_IODIRB, false);
    regBank [MCP23017_REG_GPPUA] = read (MCP23017_REG_GPPUA, false);
    regBank [MCP23017_REG_GPPUB] = read (MCP23017_REG_GPPUB, false);
    // debugPort.println (F("Success"));
    printOperationStatus (readError());
    
    if (mode == OUTPUT) { // If OUTPUT
      debugPort.println (F("pinMode(): Mode is OUTPUT"));
      // Since identical registers are paired, they sit next to each other in sequential mode,
      // shifting right 3 times is dividing by 8.
      // It finds if a number is less than or greater than 8
      //outputs 0 if number is < 8, and 1 if >= 8
      //this is used to access the correct index in the reg bank.
      //setting 1 means INPUT for MCP23017, opposite of Arduino definition
      //so to set a pin as output, we must write a single bit as 0, while other bits unaffected
      //this is done by shifting 1U to left by (pin & 0x7U) times
      //(pin & 0x7U) times because we only need to know the relative position of the bit (0-7)
      //and complementing the resulting value, for example 00100000 -> 11011111
      //resulting value is ANDed with the register value in local reg bank
      pinModeByte = regBank [pin >> 3] & (~(0x1U << (pin & 0x7U))); //write 0
    }
    else { // If INPUT
      debugPort.println (F("pinMode(): Mode is INPUT"));
      // To set as INPUT, we need to write 1.
      // This is done by ORing a 1, eg 00100000.
      pinModeByte = regBank [pin >> 3] | (0x1U << (pin & 0x7U));  // Set port IO register, write 1

      if (mode == INPUT_PULLUP) {  // Enable pull-up
        debugPort.println (F("pinMode(): With PULL-UP"));
        pullupModeByte = regBank [MCP23017_REG_GPPUA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //write 1
      }
      else {  // Disable pull-up
        pullupModeByte = regBank [MCP23017_REG_GPPUA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //write 0
      }
    }

    // Now write to the device.
    // Since we keep addresses in sequential mode by default, no need to explcitly specifiy the port B addresses.
    // Translation is done inside write function by detecting if the address we send is odd or even.
    // bankMode can tell if address translation is needed or not,
    // and this assumes false = 0, and true = 1 for the compiler.

    // Write the pin mode register.
    debugPort.println (F("Writing IODIR register"));
    response_1 = write ((MCP23017_REG_IODIRA + (pin >> 3)), pinModeByte, false); // Write single byte
    debugPort.println (F("Success"));

    if (response_1 == MCP23017_RESP_OK) {  // Save to register bank only if the response is OK
      // Since identical registers are sequentially paired, they sits next to each other.
      // But just don't write the byte you just created (Why?).
      // Do AND or OR appropreatly on the register bank value.
      // regBank [MCP23017_REG_IODIRA + (pin >> 3)] = (mode == OUTPUT) ? (regBank[MCP23017_REG_IODIRA + (pin >> 3)] & pinModeByte) : (regBank[MCP23017_REG_IODIRA + (pin >> 3)]) | pinModeByte;  //save the new value to local reg bank
      
      debugPort.println (F("Saving values"));
      regBank [MCP23017_REG_IODIRA + (pin >> 3)] = pinModeByte;
      // if (mode == OUTPUT) { // If OUTPUT
      //   regBank [MCP23017_REG_IODIRA + (pin >> 3)] = regBank[MCP23017_REG_IODIRA + (pin >> 3)] & pinModeByte;  // Write 0
      // }
      // else {  // If INPUT
      //   regBank [MCP23017_REG_IODIRA + (pin >> 3)] = regBank[MCP23017_REG_IODIRA + (pin >> 3)] | pinModeByte;  // Write 1
      // }
    }

    // Write the pull-up register value.
    // To enable it for INPUT_PULLUP or to disable it for INPUT.
    if ((mode == INPUT_PULLUP) || (mode == INPUT)) {
      debugPort.println (F("Writing GPPU register"));
      response_2 = write ((MCP23017_REG_GPPUA + (pin >> 3)), pullupModeByte, false); //write single byte
      debugPort.println (F("Success"));

      if (response_2 == MCP23017_RESP_OK) {  // Save to the register bank only if response is OK
        regBank [MCP23017_REG_GPPUA + (pin >> 3)] = pullupModeByte;
        // if (mode == INPUT_PULLUP) {
        //   regBank [MCP23017_REG_GPPUA + (pin >> 3)] = regBank [MCP23017_REG_GPPUA + (pin >> 3)] | pullupModeByte;  // Write 1
        // }
        // else {
        //   regBank [MCP23017_REG_GPPUA + (pin >> 3)] = regBank [MCP23017_REG_GPPUA + (pin >> 3)] & pullupModeByte;  // Write 0
        // }
      }
    }

    debugPort.println (F("Pin mode configured\n"));

    // Returns the largest of the error code.
    return (response_1 > response_2) ? response_1 : response_2;  // Return I2C response code
  }

  return MCP23017_ERROR_OOR;  // Address out of range
}

//============================================================================================//
/**
 * @brief Sets the IO direction of the specified port.
 * 
 * @param port The Port ID. 0 = Port A, 1 = Port B.
 * @param mode The IO direction. Can be INPUT, INPUT_PULLUP or OUTPUT.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: portMode (uint8_t port, uint8_t mode) {
  if ((port < MCP23017_PORTCOUNT) && (mode < MCP23017_PINMODES)) {
    uint8_t portModeByte = 0;
    uint8_t pullupModeByte = 0;
    uint8_t response_1, response_2 = 0;
    
    if (mode == OUTPUT) { // If OUTPUT
      portModeByte = 0; // Write 0
    }
    else { // If INPUT
      portModeByte = 0xFF;

      if (mode == INPUT_PULLUP) {  // Enable pull-up
        pullupModeByte = 0xFF;  // Write 1
      }
      else {  // Disable pull-up
        pullupModeByte = 0; // Write 0
      }
    }

    // Write the pin mode register.
    // Here, we don't have to shift right 3 times since port value is either 0 or 1.
    response_1 = write ((MCP23017_REG_IODIRA + port), portModeByte, false); // Write single byte

    if (response_1 == MCP23017_RESP_OK) {  // Save to reg bank only if response is OK
      regBank [MCP23017_REG_IODIRA + port] = portModeByte;
    }

    if (mode == INPUT_PULLUP) {
      response_2 = write ((MCP23017_REG_GPPUA + port), pullupModeByte, false); // Write single byte
      
      if (response_2 == MCP23017_RESP_OK) {
        regBank [MCP23017_REG_GPPUA + port] = pullupModeByte;
      }
    }

    // Returns the largest of the error code.
    return (response_1 > response_2) ? response_1 : response_2;  // Return I2C response code
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Writes a digital state to GPIO pins of both ports. Pin numbers can be from 0 to 15.
 * Valuesa are written to the output latch registers. If a pin is set as INPUT and you write
 * to the latch register, it will have no effect on the pin state. The latch register only
 * affects pin that are set as OUTPUT.
 * 
 * @param pin The GPIO pin number. Can be from 0 to 15.
 * @param value The state of the pin. 1 = HIGH, 0 = LOW.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: digitalWrite (uint8_t pin, uint8_t value) {
  if ((pin < MCP23017_PINCOUNT) && (value < 2)) {  // Check if values are in range
    // Read the values from the device.
    // bankMode can tell if address translation is needed or not.
    regBank [MCP23017_REG_OLATA] = read (MCP23017_REG_OLATA, false);
    regBank [MCP23017_REG_OLATB] = read (MCP23017_REG_OLATB, false);
    
    uint8_t portValueByte = 0;
    
    if (value == MCP23017_HIGH) {
      // Writing to latches will modify all output pins to the corresponding state.
      portValueByte = regBank [MCP23017_REG_OLATA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Write 1
    }
    else {
      portValueByte = regBank [MCP23017_REG_OLATA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); // Write 0
    }

    uint8_t response = write ((MCP23017_REG_OLATA + (pin >> 3)), portValueByte, false); // Write single byte

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + (pin >> 3)] = portValueByte;
    }
    // debugPort.print (F("Write error : "));
    // debugPort.println (response);
    // debugPort.println();
    return response;
  }

  return MCP23017_ERROR_OOR;  // Address out of range
}

//============================================================================================//

bool CSE_MCP23017:: printOperationStatus (bool input) {
  if (input == true) {
    debugPort.println (F("Success"));
    return true;
  }
  else {
    debugPort.println (F("Failed"));
    return false;
  }
}

//============================================================================================//
/**
 * @brief Writes to an entire port (8 pins) at once.
 * 
 * @param port The targt IO port. Can be 0 = Port A, or 1 = Port B.
 * @param value The port value.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: portWrite (uint8_t port, uint8_t value) {
  if ((port < MCP23017_PORTCOUNT) && (value < 2)) {
    uint8_t response = 0;

    // If value is 1, 0xFF will be written; o otherwise
    response = write ((MCP23017_REG_OLATA + port), (value * 0xFF), false); // Write single byte

    if (response == MCP23017_RESP_OK) {
       regBank [MCP23017_REG_OLATA + port] = (value * 0xFF);
    }

    return response;
  }

  return MCP23017_ERROR_OOR;  // Address out of range
}

//============================================================================================//
/**
 * @brief Toggles the state of an entire port.
 * 
 * @param port The target port. Can be 0 = Port A, or 1 = Port B.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: togglePort (uint8_t port) {
  if (port < MCP23017_PORTCOUNT) {
    // First read the device register
    uint8_t portValue = read ((MCP23017_REG_OLATA + port), false);
    portValue = ~(portValue); // Complement the byte

    // Only output latch register will be written
    uint8_t response = write ((MCP23017_REG_OLATA + port), portValue, false);

    // Save to the local register bank
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + port] = portValue;
    }
    return response;
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Toogles the state of a single pin.
 * 
 * @param pin The pin to toggle. Can be 0-15.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: togglePin (uint8_t pin) {
  if (pin < MCP23017_PINCOUNT) {
    // First read the device register.
    uint8_t portValue = read ((MCP23017_REG_OLATA + (pin >> 3)), false);

    // Now toggle a single bit.
    // XORing with 1 will cause the source bit to toggle.
    portValue ^= (0x1U << (pin & 0x7U));

    uint8_t response = write ((MCP23017_REG_OLATA + (pin >> 3)), portValue, false);

    // Save the value.
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + (pin >> 3)] = portValue;
    }
    return response;
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Reads a single GPIO pin.
 * 
 * @param pin The pin to read. Can be 0-15.
 * @return uint8_t The state of the pin.
 */
uint8_t CSE_MCP23017:: digitalRead (uint8_t pin) {
  if (pin < MCP23017_PINCOUNT) {
    if (readPinBit (pin, MCP23017_REG_GPIOA) == 1) {
      return MCP23017_HIGH;
    }
    else {
      return MCP23017_LOW;
    }
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Reads the mode of a single pin.
 * 
 * @param pin The pin to read. Can be 0-15.
 * @return uint8_t The mode of the pin. Can be `INPUT` (`0`), `OUTPUT` (`1`) or `INPUT_PULLUP` (`2`).
 */
uint8_t CSE_MCP23017:: readPinMode (uint8_t pin) {
  if (pin < MCP23017_PINCOUNT) {
    // Note : A 0 means Output for the IOE and 1 means Input.
    // Should check >0 since the bit can appear anywhere on the octet.
    // Example : 0b0010 0000
    if (readPinBit (pin, MCP23017_REG_IODIRA) == 1) {  // If INPUT
      if (readPinBit (pin, MCP23017_REG_GPPUA) == 1) {
        return INPUT_PULLUP;
      }
      return INPUT;
    }
    else {  // If OUTPUT
      return OUTPUT;
    }
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Reads the configuration bit associated with a single GPIO pin from any register
 * except the IOCON register.
 * 
 * @param pin The pin to read. Can be 0-15.
 * @param reg The register to read from.
 * @param translate Whether to translate the register address or not.
 * @return uint8_t The bit value.
 */
uint8_t CSE_MCP23017:: readPinBit (uint8_t pin, uint8_t reg, bool translate) {
  if ((pin < MCP23017_PINCOUNT) && (reg <= MCP23017_REGADDR_MAX)) {
    regBank [reg + (pin >> 3)] = read ((reg + (pin >> 3)), translate);
    return ((regBank [reg + (pin >> 3)] & (0x1U << (pin & 0x7U))) > 0) ? 1 : 0;
  }
}

//============================================================================================//
/**
 * @brief Read the state of the port.
 * 
 * @param port The port to read. Can be 0 = Port A, or 1 = Port B.
 * @return uint8_t The state of the port.
 */
uint8_t CSE_MCP23017:: portRead (uint8_t port) {
  if (port < MCP23017_PORTCOUNT) {
    regBank [MCP23017_REG_GPIOA + port] = read ((MCP23017_REG_GPIOA + port), false);

    return regBank [MCP23017_REG_GPIOA + port];
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Sets the GPIO input polarity. Onyl applicable for pins in INPUT mode.
 * Inverting means a LOW input will be registered as 1 and a HIGH as 0.
 * 
 * @param pin The pinn to set the polarity for.
 * @param value The polarity to set. Can be 0 = Non-inverting, or 1 = Inverting.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: setPinInputPolarity (uint8_t pin, uint8_t value) {
  if ((pin < MCP23017_PINCOUNT) && (value < 2)) {
    uint8_t regValue = 0;  // A temp byte

    //read registers
    regBank [MCP23017_REG_IPOLA] = read (MCP23017_REG_IPOLA, false);
    regBank [MCP23017_REG_IPOLB] = read (MCP23017_REG_IPOLB, false);

    if (value == 1) {  // Invert polarity
      regValue = regBank [MCP23017_REG_IPOLA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Write 1
    }
    else {  // No inversion
      regValue = regBank [MCP23017_REG_IPOLA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); // Write 0
    }

    uint8_t response = write ((MCP23017_REG_IPOLA + (pin >> 3)), regValue, false); // Write single byte

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IPOLA + (pin >> 3)] = regValue;
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Sets the input polarity for the entire port. Inverting means a LOW input will be
 * registered as 1 and a HIGH as 0.
 * 
 * @param port The port to configure. Can be 0 = Port A, or 1 = Port B.
 * @param value The polarity to set. Can be 0 = Non-inverting, or 1 = Inverting.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: setPortInputPolarity (uint8_t port, uint8_t value) {
  if ((port < MCP23017_PINCOUNT) && (value < 2)) {
    uint8_t regValue = 0;  // A temp byte
    uint8_t response = 0;

    if (value == 1) {  // Invert polarity
      regValue = 0xFF;
      response = write ((MCP23017_REG_IPOLA + port), 0xFF, false); // Write single byte
    }
    else {  // No inversion
      regValue = 0;
      response = write ((MCP23017_REG_IPOLA + port), 0x0, false); // Write single byte
    }

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IPOLA + port] = regValue;
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief Configures the output interrupt of the IO expander.
 * `attachPin` is the MCU pin to which the interrupt outputs (`INTA`, `INTB`) of the IOE are
 * connected. `outType` is the output type of the IOE's interrupt pins. Can be
 * `MCP23017_ACTIVE_LOW`, `MCP23017_ACTIVE_HIGH` or `MCP23017_OPENDRAIN`.
 * `mirror` is whether you want to activate both `INTA` and `INTB` for interrups from any of
 * the ports.
 * 
 * This function only accepts one pin.
 * 
 * @param attachPin The GPIO pin of the MCU to which the interrupt outputs are connected.
 * @param outType The output type of the IOE's interrupt pins. Can be `MCP23017_ACTIVE_LOW`, `MCP23017_ACTIVE_HIGH` or `MCP23017_OPENDRAIN`.
 * @param mirror Whether to activate both `INTA` and `INTB` for interrups from any of the ports.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: configInterrupt (int8_t attachPin, uint8_t outType, uint8_t mirror) {
  return configInterrupt (attachPin, -1, outType, mirror);
}

//--------------------------------------------------------------------------------------------//
/**
 * @brief Configures the output interrupt of the IO expander.
 * `attachPin` is the MCU pin to which the interrupt outputs (`INTA`, `INTB`) of the IOE are
 * connected. `outType` is the output type of the IOE's interrupt pins. Can be
 * `MCP23017_ACTIVE_LOW`, `MCP23017_ACTIVE_HIGH` or `MCP23017_OPENDRAIN`.
 * `mirror` is whether you want to activate both `INTA` and `INTB` for interrups from any of
 * the ports.
 * 
 * This function accepts two pins.
 * 
 * @param attachPin1 The interrupt attach pin for INTA.
 * @param attachPin2 The interrupt attach pin for INTB.
 * @param outType The output type of the IOE's interrupt pins. Can be `MCP23017_ACTIVE_LOW`, `MCP23017_ACTIVE_HIGH` or `MCP23017_OPENDRAIN`.
 * @param mirror Whether to activate both `INTA` and `INTB` for interrups from any of the ports.
 * @return uint8_t The I2C response code.
 */
uint8_t CSE_MCP23017:: configInterrupt (int8_t attachPin1, int8_t attachPin2, uint8_t outType, uint8_t mirror) {
  if ((outType < 3) && (mirror < 2)) {
    // If no pins are specified, just return
    if ((attachPin1 == -1) && (attachPin2 == -1)) {
      isIntConfigured = false;
      return MCP23017_ERROR_PAE;  // Pin assignment error
    }

    attachPinA = attachPin1;
    attachPinB = attachPin2;
    intOutType = outType;
    
    uint8_t regByte = 0;
    uint8_t response = 0;

    debugPort.println (F("Configuring host interrupt"));
    debugPort.println (F("Reading IOCON"));
    regBank [MCP23017_REG_IOCON] = read (MCP23017_REG_IOCON, false);
    debugPort.println (F("Success"));

    //--------------------------------------------------------------------------------------------//
    // Set or reset open drain first.
    // Open-drain bit overrides the other two types.

    if (outType == MCP23017_OPENDRAIN) {
      debugPort.println (F("Output type is Open Drain"));
      regByte = regBank [MCP23017_REG_IOCON] | (1U << MCP23017_BIT_ODR); // Write 1
    }
    else {
      regByte = regBank [MCP23017_REG_IOCON] & (~(1U << MCP23017_BIT_ODR)); // Write 0
    }

    //--------------------------------------------------------------------------------------------//
    // Open-drain bit has to be 0 for these to work.

    if (outType == MCP23017_ACTIVE_LOW) {
      debugPort.println (F("Output type is Active Low"));
      regByte &= (~(1U << MCP23017_BIT_INTPOL));  // Write 0
    }
    else if (outType == MCP23017_ACTIVE_HIGH) {
      debugPort.println (F("Output type is Active High"));
      regByte |= (1U << MCP23017_BIT_INTPOL); // Write 1
    }

    //--------------------------------------------------------------------------------------------//
    // Reuse regByte since we need to keep previous modifications.

    if (mirror == MCP23017_INT_MIRROR) {
      debugPort.println (F("Also mirror interrupt output"));
      regByte |= (1U << MCP23017_BIT_MIRROR); // Write 1
    }
    else {
      regByte &= (~(1U << MCP23017_BIT_MIRROR));  // Write 0
    }

    //--------------------------------------------------------------------------------------------//

    debugPort.println (F("Writing IOCON"));
    // Now write to device
    response = write (MCP23017_REG_IOCON, regByte, false); // Write single byte
    debugPort.println (F("Success\n"));

    // Save to register bank
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IOCON] = regByte;
    }

    //--------------------------------------------------------------------------------------------//
    
    if (attachHostInterrupt() == MCP23017_RESP_OK) {
      debugPort.println (F("Host MCU interrupt attach success\n"));
      isIntConfigured = true;
    }
    else {
      debugPort.println (F("Host MCU interrupt attach failed\n"));
      isIntConfigured = false;
      return MCP23017_ERROR_OF; // Operation fail
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
/**
 * @brief This function attaches an ISR to one of the GPIO pins. The ISR is called when an interrupt occurs
 * on the specified pin. The interrupt input modes can be CHANGE (1), FALLING (2), RISING (3), LOW (4) or HIGH (5).
 * Mode 0 is invalid. Note that inverting the input polarity of pins/ports will also invert the interrupts.
 * Means, an actual falling edge signal will be treated as rising edge interrupt, for example.
 * 
 * @param pin The GPIO pin to attach the ISR to.
 * @param isr An interupt service routine. Can be any valid function names.
 * @param mode The mode of the interrupt input. Can be CHANGE (1), FALLING (2), RISING (3), LOW (4) or HIGH (5).
 * @return int MCP23017_RESP_OK or MCP23017_ERROR_WF.
 */
int CSE_MCP23017:: attachInterrupt (uint8_t pin, ioeCallback_t isr, uint8_t mode) {
  if ((pin < MCP23017_PINCOUNT) && (mode <= MCP23017_INTERRUPT_COUNT)) {
    debugPort.print (F("Attaching interrupt to ioe pin "));
    debugPort.println (pin);

    if (readPinMode (pin) == OUTPUT) {
      debugPort.println (F("Pin is not configured as Input. Interrupts work only on Input pins.\n"));
      debugPort.print (F("Pin mode is "));
      debugPort.println (readPinMode (pin));
      return MCP23017_ERROR_OF;
    }

    if (isIntConfigured) { // Check if interrupt is configured
      uint8_t regByte = 0;
      uint8_t response = 0;

      // Read the registers.
      debugPort.println (F("Reading registers"));
      regBank [MCP23017_REG_IODIRA] = read (MCP23017_REG_GPINTENA, false); // Interrupt enable
      regBank [MCP23017_REG_IODIRB] = read (MCP23017_REG_GPINTENB, false);

      regBank [MCP23017_REG_GPINTENA] = read (MCP23017_REG_GPINTENA, false); 
      regBank [MCP23017_REG_GPINTENB] = read (MCP23017_REG_GPINTENB, false);
      
      regBank [MCP23017_REG_DEFVALA] = read (MCP23017_REG_DEFVALA, false); // Default compare value
      regBank [MCP23017_REG_DEFVALB] = read (MCP23017_REG_DEFVALB, false);
      
      regBank [MCP23017_REG_INTCONA] = read (MCP23017_REG_INTCONA, false); // Interrupt control
      regBank [MCP23017_REG_INTCONB] = read (MCP23017_REG_INTCONB, false);
      debugPort.println (F("Success"));

      //--------------------------------------------------------------------------------------------//
      // Needs to make INTCON bit 0, and the value in DEFVAL doesn't matter now.
      
      if (mode == MCP23017_INT_CHANGE) {
        debugPort.println (F("Setting int mode to CHANGE"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); // Set 0
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;  // If success, save the value

          isrPtrList [pin] = isr; // Save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; // Save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //--------------------------------------------------------------------------------------------//
      // Needs to set INTCON bit to 1 and DEFVAL bit to 0.
      // Interrupt occurs at the time of opposite state.
      // If DEFVAL is 0, a 0 -> 1 (rising) transition will cause an interrupt.
      
      else if (mode == MCP23017_INT_RISING) {
        debugPort.println (F("Setting int mode to RISING"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Setting DEFVAL register"));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); // Set 0
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; // Save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; // Save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //--------------------------------------------------------------------------------------------//
      // Needs to set INTCON bit to 1 and DEFVAL bit to 1.
      // Interrupt occurs at the time of opposite state.
      // If DEFVAL is 1, a 1 -> 0 (falling) transition will cause an interrupt.
      
      else if (mode == MCP23017_INT_FALLING) {
        debugPort.println (F("Setting int mode to FALLING"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Wet 1
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; // Save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; // Save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          debugPort.println (F("Interrupt configured for FALLING"));
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //--------------------------------------------------------------------------------------------//
      // Needs to set INTCON bit to 1 and DEFVAL bit to 1.
      // State checking is same as falling edge interrupt.
      
      else if (mode == MCP23017_INT_LOW) {
        debugPort.println (F("Setting int mode to LOW"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF; // Write failure
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Wet 1
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; // Save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; // Save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //--------------------------------------------------------------------------------------------//
      // Needs to set INTCON bit to 1 and DEFVAL bit to 0.
      // State checking is same as rising edge interrupt.
      
      else if (mode == MCP23017_INT_HIGH) {
        debugPort.println (F("Setting int mode to HIGH"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); // Set 0
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); // Write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; // Save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; // Save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //--------------------------------------------------------------------------------------------//
      // Invalid mode.

      else {
        debugPort.println (F("MCP23017 : Wrong interrupt mode (0). Failed to attach interrupt."));
      }

      //--------------------------------------------------------------------------------------------//

      debugPort.println (F("Enabling Interrupt on Change"));
      // Set GPINTEN to 1 enable the interrupt on change for each pin.
      regByte = regBank [MCP23017_REG_GPINTENA + (pin >> 3)] | (0x1U << (pin & 0x7U));  // Set 1
      response = write ((MCP23017_REG_GPINTENA + (pin >> 3)), regByte, false); // Write single byte

      // Save the value.
      if (response == MCP23017_RESP_OK) {
        debugPort.println (F("Success\n"));
        regBank [MCP23017_REG_GPINTENA + (pin >> 3)] = regByte;
        return MCP23017_RESP_OK;
      }
      else {
        return MCP23017_ERROR_WF;
      }
    }
    else {
      debugPort.println (F("MCP23017 : Interrupt is not configured. Use configInterrupt() to configure.\n"));
    }
  }

  return MCP23017_ERROR_OOR;
}

//============================================================================================//
//attach the host MCU interrupt to detect interrupt output from the IO expander
//use configInterrupt to configure the pins and type
/**
 * @brief Attachs the host MCU interrupt to detect the outputs from the IO expander.
 * Use `configInterrupt()` to configure the pins and type.
 * 
 * @return uint8_t The status.
 */
uint8_t CSE_MCP23017:: attachHostInterrupt() {
  debugPort.println (F("Attaching host MCU interrupt"));
  // Active-Low means the signal will be a falling edge.
  if (intOutType == MCP23017_ACTIVE_LOW) {
    debugPort.println (F("Output is Active Low"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      delay (100);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeIndex], FALLING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { // B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      delay (100);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeIndex], FALLING);
      debugPort.println (F("Success"));
    }
  }

  // Active-High means the signal will be a rising edge.
  else if (intOutType == MCP23017_ACTIVE_HIGH) {
    debugPort.println (F("Output is Active High"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeIndex], RISING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { // B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeIndex], RISING);
      debugPort.println (F("Success"));
    }
  }

  // Open-Drain means the interrupt output from the IOE will be either LOW or High-Z.
  // If you choose Opne-Drain, you have to either use the  internal pull-up of the host MCU
  // or add an external pull-up. That means the circuit will be equivalent to that of falling edge detection.
  else if (intOutType == MCP23017_OPENDRAIN) {
    debugPort.println (F("Output is Open Drain"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeIndex], FALLING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { // B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeIndex);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeIndex], FALLING);
      debugPort.println (F("Success"));
    }
  }

  else {
    debugPort.print (F("Failed\n"));
    return MCP23017_ERROR_OOR;
  }

  debugPort.println();

  return MCP23017_RESP_OK;
}

//============================================================================================//
/**
 * @brief Processes an interrupt from the IO expander.
 * 
 */
void CSE_MCP23017:: dispatchInterrupt() {
  if ((interruptActive == true) && (stateReverted == true)) {
    isrSupervisor();
    interruptActive = false;
    debugPort.println (F("Interrupt flag has been reset\n"));
    delay (100);
    regBank [MCP23017_REG_INTCAPA] = read (MCP23017_REG_INTCAPA, false);
    regBank [MCP23017_REG_INTCAPB] = read (MCP23017_REG_INTCAPB, false);
    stateReverted = false;
  }

  else if (stateReverted == false) {
    if (lastIntPin >= 0) {
      if (isrModeList [lastIntPin] == MCP23017_INT_FALLING) {
        if (readPinBit (lastIntPin, MCP23017_REG_GPIOA) == MCP23017_HIGH) {
          stateReverted = true;
        }
      }
      else if (isrModeList [lastIntPin] == MCP23017_INT_RISING) {
        if (readPinBit (lastIntPin, MCP23017_REG_GPIOA) == MCP23017_LOW) {
          stateReverted = true;
        }
      }
    }
  }
}

//============================================================================================//

bool CSE_MCP23017:: interruptPending() {
  return interruptActive;
}

//============================================================================================//
/**
 * @brief This is the supervisor function that manages all user ISRs. When an interrupt is registered
 * by the host MCU, the `isrSupervisor()` should determine which pin of the IOE caused it and invoke the
 * associated ISR set by the user. This requires disabling the host MCU interrupt first and then reading
 * the IOE registers INTF, GPINTEN, DEFVAL, INTCON, and INTCAP in order. Then determine which pin caused
 * the interrupt, verify the interrupt attached to the pin, and call the ISR.
 * On return, re-attach the host MCU interrupt.
 * 
 */
void CSE_MCP23017:: isrSupervisor() {
  if (isIntConfigured == true) { // Not necessary though
    // Detach the host MCU interrupts.
    if (attachPinA != -1) {
      ::detachInterrupt (attachPinA); // Arduino-API
    }
    
    if (attachPinB != -1) {
      ::detachInterrupt (attachPinB);
    }
    // debugPort.println (F("Detaching host interrupts"));
  }

  //--------------------------------------------------------------------------------------------//
  // Read the registers.

  // The interrupt flag determines which pin caused the interrupt.
  // This has to be read before disabling the IOE interrupts.
  regBank [MCP23017_REG_INTFA] = read (MCP23017_REG_INTFA, false);
  regBank [MCP23017_REG_INTFB] = read (MCP23017_REG_INTFB, false);

  regBank [MCP23017_REG_GPINTENA] = read (MCP23017_REG_GPINTENA, false); // Interrupt enable
  regBank [MCP23017_REG_GPINTENB] = read (MCP23017_REG_GPINTENB, false);

  write (MCP23017_REG_GPINTENA, 0);
  write (MCP23017_REG_GPINTENB, 0);

  // regBank [MCP23017_REG_INTCONA] = read (MCP23017_REG_INTCONA, false); // IOC control
  // regBank [MCP23017_REG_INTCONB] = read (MCP23017_REG_INTCONB, false);

  // write (MCP23017_REG_INTCONA, 0);
  // write (MCP23017_REG_INTCONB, 0);

  debugPort.println (F("ISR Supervisor invoked"));
  debugPort.println (F("Disabling IOE interrupts"));

  if ((!readError()) || (!writeError())) {
    debugPort.println (F("Success"));
  }
  else {
    debugPort.println (F("Failed"));
  }

  debugPort.println (F("Reading registers"));
  
  // regBank [MCP23017_REG_DEFVALA] = read (MCP23017_REG_DEFVALA, false); // Default compare value
  // regBank [MCP23017_REG_DEFVALB] = read (MCP23017_REG_DEFVALB, false);

  regBank [MCP23017_REG_INTCAPA] = read (MCP23017_REG_INTCAPA, false); // Interrupt capture
  regBank [MCP23017_REG_INTCAPB] = read (MCP23017_REG_INTCAPB, false);

  if (!readError()) {
    debugPort.println (F("Success"));
  }
  else {
    debugPort.println (F("Failed"));
  }
  
  debugPort.print (F("MCP23017_REG_INTFA : 0x"));
  debugPort.print (this->regBank [MCP23017_REG_INTFA], HEX);
  debugPort.print (F(", 0b"));
  debugPort.println (toBinary (this->regBank [MCP23017_REG_INTFA], 8));
  debugPort.print (F("MCP23017_REG_INTFB : 0x"));
  debugPort.print (this->regBank [MCP23017_REG_INTFB], HEX);
  debugPort.print (F(", 0b"));
  debugPort.println (toBinary (this->regBank [MCP23017_REG_INTFB], 8));

  debugPort.print (F("INTCAPA : 0x"));
  debugPort.print (this->regBank [MCP23017_REG_INTCAPA], HEX);
  debugPort.print (F(", 0b"));
  debugPort.println (toBinary (this->regBank [MCP23017_REG_INTCAPA], 8));
  debugPort.print (F("INTCAPB : 0x"));
  debugPort.print (this->regBank [MCP23017_REG_INTCAPB], HEX);
  debugPort.print (F(", 0b"));
  debugPort.println (toBinary (this->regBank [MCP23017_REG_INTCAPB], 8));

  //--------------------------------------------------------------------------------------------//

  intPin = -1;
  intPinState = -1;
  intPinCapState = -1;

  // We have two bytes to check. We expect only one bit of either the registers to be a 1.
  // To find where the bit 1 is, Rsh the bytes 1 bit at a time, and compare it with 0x1.
  // If it is 1, the final position index will be the pin location.
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 8; j++) {
      if (((regBank [MCP23017_REG_INTFA + i] >> j) & 0x1U) == 0x1U) {
        intPin = int8_t (j + (8 * i));
        break;
      }
    }
    if (intPin != -1) break;
  }

  debugPort.print (F("Interrupt occured at "));
  debugPort.println (intPin);

  //--------------------------------------------------------------------------------------------//

  if (intPin == -1) { // If the pins couldn't be determined
    debugPort.println (F("MCP23017 Error : Unable to determine the pin interrupt occured at\n"));
    // return MCP23017_ERROR_UDP;
    // return;
  }
  else {
    lastIntPin = intPin;
    // Read the pin state when interrupt occured.
    // intPinCapState = regBank[MCP23017_REG_INTCAPA + (intPin >> 3)] & (0x1U << (intPin & 0x7));
    intPinCapState = (regBank [MCP23017_REG_INTCAPA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U;
      
    //--------------------------------------------------------------------------------------------//
    // If the interrupt is set for LOW state, we have to read the port register every time
    // after the ISR is finished and call it again if the state persists.
    
    if (isrModeList [intPin] == MCP23017_INT_LOW) {
      if (intPinCapState == 0) { // If the bit pos is 0
        bool statePersist = false;

        do {
          isrPtrList [intPin] (intPin);  // Call the ISR attached to the pin

          read (regBank [MCP23017_REG_GPIOA + (intPin >> 3)]);  // Read just the associated reg only to save time
          intPinState = regBank [MCP23017_REG_GPIOA + (intPin >> 3)] & (0x1U << (intPin & 0x7));

          if (intPinState == 0) { // If the state persists
            statePersist = true;
          }
        } while (statePersist == true); // Repeat
      }
    }

    //--------------------------------------------------------------------------------------------//
    // If the interrupt is set for HIGH state, we have to read the port register every time
    // after the ISR is finished and call it again if the state persists.
    
    else if (isrModeList [intPin] == MCP23017_INT_HIGH) {
      // The bit that is 1 may appear anywhere on the byte. So we just need to check if the result if > 0.
      // Instead of shifting 1U to arbitrary left, the reg value can itself be shifted to right as,
      // (regBank[MCP23017_REG_INTCAPA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U.
      // This is only valid for reading the bit pos in a reg.
      if (intPinCapState == 1) { // If the bit pos is 1
        bool statePersist = false;
        
        do {
          isrPtrList [intPin] (intPin);  // Call the ISR attached to the pin

          read (regBank [MCP23017_REG_GPIOA + (intPin >> 3)]);  // Read just the associated reg only to save time
          intPinState = (regBank [MCP23017_REG_GPIOA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U;
          
          if (intPinState == 1) { // If the state persists
            statePersist = true;
          }
        } while (statePersist == true); // Repeat
      }
    }

    //--------------------------------------------------------------------------------------------//
    // If the interrupt is set for CHANGE of state, then we do not need check any registers.
    // Because the interrupt could have occured when a state of change occured and it occurs only once.
    
    else if (isrModeList [intPin] == MCP23017_INT_CHANGE) {
      isrPtrList [intPin] (intPin);  // Call the ISR attached to the pin
    }

    //--------------------------------------------------------------------------------------------//
    // This is simlar to LOW state interrupt except the ISR is called only once.

    else if (isrModeList [intPin] == MCP23017_INT_FALLING) {
      if (intPinCapState == 0) { // If the bit pos is 0, that means the pin state changed from HIGH -> LOW
        isrPtrList [intPin] (intPin);  // Call the ISR attached to the pin
      }
    }

    //--------------------------------------------------------------------------------------------//
    // This is simlar to HIGH state interrupt except the ISR is called only once.

    else if (isrModeList [intPin] == MCP23017_INT_RISING) {
      if (intPinCapState == 1) { // If the bit pos is 0, that means the pin state changed from HIGH -> LOW
        isrPtrList [intPin] (intPin);  // Call the ISR attached to the pin
      }
    }

    //--------------------------------------------------------------------------------------------//

    else {
      debugPort.println (F("MCP23017 Error : No suitable ISRs found."));
      return;
    }
  }

  write (MCP23017_REG_GPINTENA, regBank [MCP23017_REG_GPINTENA]);
  write (MCP23017_REG_GPINTENB, regBank [MCP23017_REG_GPINTENB]);

  if (!writeError()) {
    debugPort.println (F("IOE interrupts have been re-attached"));
  } else {
    debugPort.println (F("Failed to re-attach IOE interrupts"));
  }

  if (attachHostInterrupt() == MCP23017_RESP_OK) {
    isIntConfigured = true;
    debugPort.println (F("Host interrupt has been re-attached"));
    return;
  }
  else {
    isIntConfigured = false;
    return;
  }
    // debugPort.println (F("Interrupt has been served"));
}

//===============================================================================//
/**
 * @brief Reverses an ASCII formatted binary number string.
 * 
 * @param sourceString The string to be reversed.
 */
void reverseString (char *sourceString) { // Modify the source string
  uint8_t temp;  // Copy the char
  int32_t arraySize = strlen (sourceString);

  for (int i = 0; i < (arraySize / 2); i++) {
    temp = sourceString [i]; // Copy the char
    sourceString [i] = sourceString [(arraySize - 1) - i]; // Switch the digits from right to left
    sourceString [(arraySize - 1) - i] = temp; // Copy the left half char to right
  }
}

//===============================================================================//
/**
 * @brief Converts a number to its binary formatted string.
 *  If the original binary of the number is less than the width, the remaining
 * positions will be filled with zeros.
 * 
 * @param number The number to convert.
 * @param width The minimum length of the string.
 * @return String The result string.
 */
String toBinary (uint64_t number, uint16_t width = 0) {
  uint8_t i = 0;
  uint8_t j = 0;
  char binaryBuffer [65] = {0}; // 65 bits - why odd number is because we can split the binary string half

  while ((number > 0) || (j < width)) { // Loop until number becomes 0
    sprintf (&binaryBuffer [i++], "%u", (number & 1)); // AND each LSB with 1, don't use %llu
    j++;
    number >>= 1; // Shift the number to right
  }

  reverseString (binaryBuffer); // Reverse the string

  // debugPort.print (F("\nbinaryBuffer : "));
  // for (int k = 0; k < j; k++) {
  //   debugPort.print (binaryBuffer [k]);
  // }

  return (String (binaryBuffer));
}

//===============================================================================//


