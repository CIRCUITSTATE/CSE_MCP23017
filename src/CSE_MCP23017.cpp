
//===================================================================================//
// Includes

#include "CSE_MCP23017.h"

//===================================================================================//
// Globals

CSE_MCP23017* ioeList [MCP23017_MAX_OBJECT] = {0};  // Pointers to all IO expander objects
uint8_t ioeCount = 0;  // IO exapander object count

//===================================================================================//
// Templates

template <int index>  //receives the index to the global arrays
void callback() {
  if (ioeList [index] != nullptr) {  //if the pointer is not zero
    if (!ioeList [index]->interruptPending()) { //if an interupt is not already being served
      debugPort.print (F("Callback invoked at "));
      debugPort.println (index);
      debugPort.print (F("Object is at 0x"));
      debugPort.println (uint32_t (ioeList [index]), HEX);
      debugPort.println();

      //activate the interrupt so that next time the ISR dispatcher is called
      //the ISR will be executed
      ioeList [index]->interruptActive = true;
      return;
    }
    else {
      debugPort.print (F("\n--\nInterrupt pending at "));
      debugPort.println (index);
      debugPort.println (F("--\n"));
      return;
    }
  }
  else {
    debugPort.print (F("Callback error at "));
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


//===================================================================================//

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
  ioeId = ioeCount;

  // Save the obj ptr to the global list, and update obj count
  ioeList [ioeCount++] = this;
}

//-----------------------------------------------------------------------------------//
//destructor

CSE_MCP23017:: ~CSE_MCP23017() {
}

//===================================================================================//
//hardware resets the IO expander
//resets the local register bank and other flags to default state

void CSE_MCP23017:: reset() {
  bankMode = PAIR;
  addressMode = 0;

  for (int i = 0; i <= MCP23017_REGADDR_MAX; i++) {
    regBank [i] = 0;
  }
  
  regBank [MCP23017_REG_IODIRA] = 0xFF; //reset values
  regBank [MCP23017_REG_IODIRB] = 0xFF;
  
  ::pinMode (resetPin, OUTPUT);
  
  ::digitalWrite (resetPin, LOW);
  ::delay (10);
  ::digitalWrite (resetPin, HIGH);
}

//===================================================================================//
//check if the device is present
//hardware resets the device

uint8_t CSE_MCP23017:: begin() {
  Wire.beginTransmission (deviceAddress);
  uint8_t response = Wire.endTransmission(); //read ACK
  
  if (response == 0) {
    debugPort.println (F("MCP23017 found\n"));
  }
  else {
    debugPort.println (F("MCP23017 not found\n"));
  }

  reset();
  return response;
}

//===================================================================================//
//writes a continous sequence of bytes from a buffer directly to the I2C peripheral
//also saves the values to the local register bank
//if translateAddress is false, absolute register address will be used

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

//-----------------------------------------------------------------------------------//
//writes a single value to the device and save the value to register bank
//it doesn't matter whether it is byte mode or sequential mode when writing single byte

uint8_t CSE_MCP23017:: write (uint8_t regAddress, uint8_t byteOne, bool translateAddress) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    Wire.beginTransmission (deviceAddress); //write device address

    if (translateAddress) {  //if bankmode = 1 (group)
      Wire.write (TRANSLATE (regAddress));  //translate address for both port A and B
    }
    else {
      Wire.write (regAddress);
    }

    Wire.write (byteOne);
    uint8_t response = Wire.endTransmission();

    if (response != MCP23017_RESP_OK) {
      writeError (true);
    }

    return response;
  }

  writeError (true);
  return MCP23017_ERROR_OOR;  //address out of range
}

//-----------------------------------------------------------------------------------//
//updates all device registers with reg bank values

uint8_t CSE_MCP23017:: write (bool translateAddress) {
  Wire.beginTransmission (deviceAddress);
  Wire.write (TRANSLATE (MCP23017_REG_IODIRA));
  
  for (int i = 0; i <= MCP23017_REGADDR_MAX; i++) {
    if (translateAddress) {
      //in bank mode, or when address translation needed,
      //we need to fetch from the correct locations in the local reg bank
      //writes to device sequentially, addressMode should be 0
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

//===================================================================================//
//reads single byte from device register

//TODO : add functions to read multiple values in single go

uint8_t CSE_MCP23017:: read (uint8_t regAddress, bool translateAddress) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    Wire.beginTransmission (deviceAddress); //write device address
    if (translateAddress) {  //if bankmode = 1 (group)
      Wire.write( TRANSLATE (regAddress));  //translate address for both port A and B
    }
    else {
      Wire.write (regAddress);
    }
    Wire.endTransmission();

    Wire.requestFrom (deviceAddress, 1, true); //address, quantity and bus release

    if (Wire.available() == 1) {
      return uint8_t (Wire.read());
    }
    
    debugPort.print (F("Device 0x"));
    debugPort.print (deviceAddress, HEX);
    debugPort.println (F(" not responding"));
    readError (true);
    return 0xFF;
  }

  debugPort.println (F("MCP23017 Error : Value out of range"));
  return MCP23017_ERROR_OOR;  //address out of range
}

//===================================================================================//
//returns the error state of a I2C read operation
//error state is reset after read
//successful read operations will not reset a previously set error state
//means the error state will persist until it is read by this function

bool CSE_MCP23017:: readError() {
  if (deviceReadError) {
    deviceReadError = false;  //reset error state
    return true;
  }
  return false;
}

//===================================================================================//
//sets the error state for read operations
//should always pass the state to avoid ambiguity

void CSE_MCP23017:: readError (bool e) {
  deviceReadError = e;
}

//===================================================================================//
//just like the readError, but for write operations
//the first call to this function will reset a previously set error state

bool CSE_MCP23017:: writeError() {
  if (deviceWriteError) {
    deviceWriteError = false; //reset error state
    return true;
  }
  return false;
}

//===================================================================================//
//sets the error state for the write operations
//should pass the state to avoid ambiguity

void CSE_MCP23017:: writeError (bool e) {
  deviceWriteError = e;
}

//===================================================================================//

uint8_t CSE_MCP23017:: readRegisters (bool translateAddress) {
  Wire.beginTransmission (deviceAddress); //write device address
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

//===================================================================================//
//updates the local register bank
//so that you can write all the values to the device in one go

uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t *buffer, uint8_t bufferOffset, uint8_t length) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    for (uint8_t i = bufferOffset; i < (bufferOffset + length); i++) {
      regBank [regAddress++] = buffer [i];
    }
    return MCP23017_RESP_OK;
  }
  return MCP23017_ERROR_OOR;
}

//-----------------------------------------------------------------------------------//
//accepts just two bytes

uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t byteOne, uint8_t byteTwo) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    regBank [regAddress] = byteOne;
    regBank [regAddress+1] = byteTwo;
    return MCP23017_RESP_OK;
  }

  return MCP23017_ERROR_OOR;
}

//-----------------------------------------------------------------------------------//
//accepts just one byte

uint8_t CSE_MCP23017:: update (uint8_t regAddress, uint8_t byteOne) {
  if (regAddress <= MCP23017_REGADDR_MAX) {  //check if address is in range
    regBank [regAddress] = byteOne;
    return MCP23017_RESP_OK;
  }
  
  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//sets the GPIO direction of both ports A and B
//pin numbers can be 0-15
//first 8 pins belongs to port A (0-7) and the rest to port B (8-15)
//modes can be INPUT (0), OUTPUT (1) or INPUT_PULLUP (2)
//but for the IO expander, 1 means INPUT and 0 means OUTPUT
//the function first writes to the device and if the operation is successful,
//the values are stored in the local register bank.
//returns the write operation response

uint8_t CSE_MCP23017:: pinMode (uint8_t pin, uint8_t mode) {
  if ((pin < MCP23017_PINCOUNT) && (mode < MCP23017_PINMODES)) {  //check if values are in range
    //first save the new register value to a temp var
    //so that if the write fails for any reason, we can retain the original value in the local register bank
    
    uint8_t pinModeByte = 0;
    uint8_t pullupModeByte = 0;
    uint8_t response_1, response_2 = 0;

    debugPort.print (F("Setting pin mode at "));
    debugPort.println (pin);
    debugPort.println (F("Reading registers"));
    
    //read the values from the device and store it at local reg bank
    //bankMode can tell if address translation is needed
    regBank [MCP23017_REG_IODIRA] = read (MCP23017_REG_IODIRA, false);
    regBank [MCP23017_REG_IODIRB] = read (MCP23017_REG_IODIRB, false);
    regBank [MCP23017_REG_GPPUA] = read (MCP23017_REG_GPPUA, false);
    regBank [MCP23017_REG_GPPUB] = read (MCP23017_REG_GPPUB, false);
    // debugPort.println (F("Success"));
    printOperationStatus (readError());
    
    if (mode == OUTPUT) { //if OUTPUT
      debugPort.println (F("Mode is OUTPUT"));
      //since identical registers are paired, they sit next to each other
      //shifting right 3 times is dividing by 8
      //it finds if a number is less than or greater than 8
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
    else { //if INPUT
      debugPort.println (F("Mode is INPUT"));
      //to set as INPUT, we need to write 1
      //this is done by ORing a 1, eg 00100000
      pinModeByte = regBank [pin >> 3] | (0x1U << (pin & 0x7U));  //set port IO register, write 1

      if (mode == INPUT_PULLUP) {  //enable pull-up
        debugPort.println (F("With PULL-UP"));
        pullupModeByte = regBank [MCP23017_REG_GPPUA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //write 1
      }
      else {  //disable pull-up
        pullupModeByte = regBank [MCP23017_REG_GPPUA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //write 0
      }
    }

    //then write to device
    //since we keep addresses in pair mode by default, no need to explcitly specifiy the port B addresses
    //translation is done inside write function by detecting if the address we send is odd or even
    //bankMode can tell if address translation is needed or not
    //and this assumes false = 0, and true = 1 for the compiler

    //write pin mode register
    debugPort.println (F("Writing IODIR register"));
    response_1 = write ((MCP23017_REG_IODIRA + (pin >> 3)), pinModeByte, false); //write single byte
    debugPort.println (F("Success"));

    if (response_1 == MCP23017_RESP_OK) {  //save to reg bank only if response is ok
      //since identical registers are paired, they sits next to each other
      //but just don't write the byte you just created (why?)
      //do AND or OR appropreatly on the reg bank value
      // regBank [MCP23017_REG_IODIRA + (pin >> 3)] = (mode == OUTPUT) ? (regBank[MCP23017_REG_IODIRA + (pin >> 3)] & pinModeByte) : (regBank[MCP23017_REG_IODIRA + (pin >> 3)]) | pinModeByte;  //save the new value to local reg bank
      
      debugPort.println (F("Saving values"));
      regBank [MCP23017_REG_IODIRA + (pin >> 3)] = pinModeByte;
      // if (mode == OUTPUT) { //if OUTPUT
      //   regBank [MCP23017_REG_IODIRA + (pin >> 3)] = regBank[MCP23017_REG_IODIRA + (pin >> 3)] & pinModeByte;  //write 0
      // }
      // else {  //if INPUT
      //   regBank [MCP23017_REG_IODIRA + (pin >> 3)] = regBank[MCP23017_REG_IODIRA + (pin >> 3)] | pinModeByte;  //write 1
      // }
    }

    //write pull-up register value
    //to enable it for INPUT_PULLUP or to disable it for INPUT
    if ((mode == INPUT_PULLUP) || (mode == INPUT)) {
      debugPort.println (F("Writing GPPU register"));
      response_2 = write ((MCP23017_REG_GPPUA + (pin >> 3)), pullupModeByte, false); //write single byte
      debugPort.println (F("Success"));

      if (response_2 == MCP23017_RESP_OK) {  //save to reg bank only if response is ok
        regBank [MCP23017_REG_GPPUA + (pin >> 3)] = pullupModeByte;
        // if (mode == INPUT_PULLUP) {
        //   regBank [MCP23017_REG_GPPUA + (pin >> 3)] = regBank [MCP23017_REG_GPPUA + (pin >> 3)] | pullupModeByte;  //write 1
        // }
        // else {
        //   regBank [MCP23017_REG_GPPUA + (pin >> 3)] = regBank [MCP23017_REG_GPPUA + (pin >> 3)] & pullupModeByte;  //write 0
        // }
      }
    }

    debugPort.println (F("Pin mode configured\n"));
    //returns the largest of the error code
    return (response_1 > response_2) ? response_1 : response_2;  //return I2C response code
  }

  return MCP23017_ERROR_OOR;  //address out of range
}

//===================================================================================//
//sets the IO direction of either ports
//port can be 0 = port A, and 1 = port B
//mode can be INPUT, INPUT_PULLUP or OUTPUT

uint8_t CSE_MCP23017:: portMode (uint8_t port, uint8_t mode) {
  if ((port < MCP23017_PORTCOUNT) && (mode < MCP23017_PINMODES)) {
    uint8_t portModeByte = 0;
    uint8_t pullupModeByte = 0;
    uint8_t response_1, response_2 = 0;
    
    if (mode == OUTPUT) { //if OUTPUT
      portModeByte = 0; //write 0
    }
    else { //if INPUT
      portModeByte = 0xFF;

      if (mode == INPUT_PULLUP) {  //enable pull-up
        pullupModeByte = 0xFF;  //write 1
      }
      else {  //disable pull-up
        pullupModeByte = 0; //write 0
      }
    }

    //write pin mode register
    //here we don't have to shift right 3 times since port value is either 0 or 1
    response_1 = write ((MCP23017_REG_IODIRA + port), portModeByte, false); //write single byte

    if (response_1 == MCP23017_RESP_OK) {  //save to reg bank only if response is ok
      regBank [MCP23017_REG_IODIRA + port] = portModeByte;
    }

    if (mode == INPUT_PULLUP) {
      response_2 = write ((MCP23017_REG_GPPUA + port), pullupModeByte, false); //write single byte
      
      if (response_2 == MCP23017_RESP_OK) {
        regBank [MCP23017_REG_GPPUA + port] = pullupModeByte;
      }
    }

    //returns the largest of the error code
    return (response_1 > response_2) ? response_1 : response_2;  //return I2C response code
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//writes a digital state to both ports A and B
//pin numbers can be 0-15
//first 8 pins belongs to port A (0-7) and the rest to port B (8-15)
//value can be HIGH (1) or LOW (0)
//values are written to the output latch registers
//if a pin has been set as INPUT and you write to the latch register,
//it will have no effect on the pin state
//the latch register state only affects the pins set as OUTPUT

uint8_t CSE_MCP23017:: digitalWrite (uint8_t pin, uint8_t value) {
  if ((pin < MCP23017_PINCOUNT) && (value < 2)) {  //check if values are in range
    //read the values from the device
    //bankMode can tell if address translation is needed
    regBank [MCP23017_REG_OLATA] = read (MCP23017_REG_OLATA, false);
    regBank [MCP23017_REG_OLATB] = read (MCP23017_REG_OLATB, false);
    
    uint8_t portValueByte = 0;
    
    if (value == MCP23017_HIGH) {
      //writing to latches will modifies all output pins to the correcponding state
      portValueByte = regBank [MCP23017_REG_OLATA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //write 1
    }
    else {
      portValueByte = regBank [MCP23017_REG_OLATA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //write 0
    }

    uint8_t response = write ((MCP23017_REG_OLATA + (pin >> 3)), portValueByte, false); //write single byte

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + (pin >> 3)] = portValueByte;
    }
    // debugPort.print(F("Write error : "));
    // debugPort.println(response);
    // debugPort.println();
    return response;
  }

  return MCP23017_ERROR_OOR;  //address out of range
}

//===================================================================================//

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

//===================================================================================//
//writes to an entire port (8 pins)
//port can be 0 = port A, and 1 = port B
//value can be 1 = _HIGH, 0 = _LOW

uint8_t CSE_MCP23017:: portWrite (uint8_t port, uint8_t value) {
  if ((port < MCP23017_PORTCOUNT) && (value < 2)) {
    uint8_t response = 0;

    //if value is 1, 0xFF will be written; o otherwise
    response = write ((MCP23017_REG_OLATA + port), (value * 0xFF), false); //write single byte

    if (response == MCP23017_RESP_OK) {
       regBank [MCP23017_REG_OLATA + port] = (value * 0xFF);
    }

    return response;
  }

  return MCP23017_ERROR_OOR;  //address out of range
}

//===================================================================================//
//toggles the entire port
//port can be 0 = port A, and 1 = port B

uint8_t CSE_MCP23017:: togglePort (uint8_t port) {
  if (port < MCP23017_PORTCOUNT) {
    //first read the device register
    uint8_t portValue = read ((MCP23017_REG_OLATA + port), false);
    portValue = ~(portValue); //complement the byte

    //only output latch register will be written
    uint8_t response = write ((MCP23017_REG_OLATA + port), portValue, false);

    //save to local reg bank
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + port] = portValue;
    }
    return response;
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//toggles the state of a single pin
//pin can be 0-15

uint8_t CSE_MCP23017:: togglePin (uint8_t pin) {
  if (pin < MCP23017_PINCOUNT) {
    //first read the device register
    uint8_t portValue = read ((MCP23017_REG_OLATA + (pin >> 3)), false);

    //now toggle a single bit
    //XORing with 1 will cause the source bit to toggle
    portValue ^= (0x1U << (pin & 0x7U));

    uint8_t response = write ((MCP23017_REG_OLATA + (pin >> 3)), portValue, false);

    //save the value
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_OLATA + (pin >> 3)] = portValue;
    }
    return response;
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//reads the state of a pin
//pin can be 0-15
//return either 1 or 0

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

//===================================================================================//

uint8_t CSE_MCP23017:: readPinMode (uint8_t pin) {
  if (pin < MCP23017_PINCOUNT) {
    //note : a 0 means output for ioe and 1 means input
    //should check >0 since the bit can appear anywhere on the octet
    //example : 0b0010 0000
    if (readPinBit (pin, MCP23017_REG_IODIRA) == 1) {  //if INPUT
      if (readPinBit (pin, MCP23017_REG_GPPUA) == 1) {
        return INPUT_PULLUP;
      }
      return INPUT;
    }
    else {  //if OUTPUT
      return OUTPUT;
    }
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//reads the bit value for a pin on a register
//only applicable to registers other than MCP23017_REG_IOCON

uint8_t CSE_MCP23017:: readPinBit (uint8_t pin, uint8_t reg, bool translate) {
  if ((pin < MCP23017_PINCOUNT) && (reg <= MCP23017_REGADDR_MAX)) {
    regBank[reg + (pin >> 3)] = read ((reg + (pin >> 3)), translate);
    return ((regBank [reg + (pin >> 3)] & (0x1U << (pin & 0x7U))) > 0) ? 1 : 0;
  }
}

//===================================================================================//
//reads the on of the ports
//port can be 0 = A, 1 = B

uint8_t CSE_MCP23017:: portRead (uint8_t port) {
  if (port < MCP23017_PORTCOUNT) {
    regBank [MCP23017_REG_GPIOA + port] = read ((MCP23017_REG_GPIOA + port), false);

    return regBank [MCP23017_REG_GPIOA + port];
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//sets the GPIO input polarity
//inverting means a LOW input will be registered as 1 and a HIGH as 0
//pin can be 0-15
//value is either 0 (no inverting) or 1 (inverting)

uint8_t CSE_MCP23017:: setPinInputPolarity (uint8_t pin, uint8_t value) {
  if ((pin < MCP23017_PINCOUNT) && (value < 2)) {
    uint8_t regValue = 0;  //a temp byte

    //read registers
    regBank [MCP23017_REG_IPOLA] = read (MCP23017_REG_IPOLA, false);
    regBank [MCP23017_REG_IPOLB] = read (MCP23017_REG_IPOLB, false);

    if (value == 1) {  //invert polarity
      regValue = regBank [MCP23017_REG_IPOLA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //write 1
    }
    else {  //no inversion
      regValue = regBank [MCP23017_REG_IPOLA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //write 0
    }

    uint8_t response = write ((MCP23017_REG_IPOLA + (pin >> 3)), regValue, false); //write single byte

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IPOLA + (pin >> 3)] = regValue;
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//sets the GPIO input polarity
//inverting means a LOW input will be registered as 1 and a HIGH as 0
//pin can be 0-15
//value is either 0 (no inverting) or 1 (inverting)

uint8_t CSE_MCP23017:: setPortInputPolarity (uint8_t port, uint8_t value) {
  if ((port < MCP23017_PINCOUNT) && (value < 2)) {
    uint8_t regValue = 0;  //a temp byte
    uint8_t response = 0;

    if (value == 1) {  //invert polarity
      regValue = 0xFF;
      response = write ((MCP23017_REG_IPOLA + port), 0xFF, false); //write single byte
    }
    else {  //no inversion
      regValue = 0;
      response = write ((MCP23017_REG_IPOLA + port), 0x0, false); //write single byte
    }

    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IPOLA + port] = regValue;
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//configure the interrupt feature of MCP23017
//attachPin is the MCU pin to which the interrupt outputs (INTA, INTB) of MCP23017
//are connected.
//outType is the output type of MCP23017's interrupt pins
//can be MCP23017_ACTIVE_LOW, MCP23017_ACTIVE_HIGH or MCP23017_OPENDRAIN
//mirror is whether you want to activate both INTA and INTB for interrups from any
//of the ports

uint8_t CSE_MCP23017:: configInterrupt (int8_t attachPin, uint8_t outType, uint8_t mirror) {
  return configInterrupt (attachPin, -1, outType, mirror);
}

//-----------------------------------------------------------------------------------//

uint8_t CSE_MCP23017:: configInterrupt (int8_t attachPin1, int8_t attachPin2, uint8_t outType, uint8_t mirror) {
  if ((outType < 3) && (mirror < 2)) {
    //if no pins are specified, just return
    if ((attachPin1 == -1) && (attachPin2 == -1)) {
      isIntConfigured = false;
      return MCP23017_ERROR_PAE;  //pin assignment error
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

    //-----------------------------------------------------------------------------------//
    //set or reset open drain first
    //open drain bit overrides the other two types

    if (outType == MCP23017_OPENDRAIN) {
      debugPort.println (F("Output type is Open Drain"));
      regByte = regBank [MCP23017_REG_IOCON] | (1U << MCP23017_BIT_ODR); //write 1
    }
    else {
      regByte = regBank [MCP23017_REG_IOCON] & (~(1U << MCP23017_BIT_ODR)); //write 0
    }

    //-----------------------------------------------------------------------------------//
    //open drain bit has to be 0 for these to work

    if (outType == MCP23017_ACTIVE_LOW) {
      debugPort.println (F("Output type is Active Low"));
      regByte &= (~(1U << MCP23017_BIT_INTPOL));  //write 0
    }
    else if (outType == MCP23017_ACTIVE_HIGH) {
      debugPort.println (F("Output type is Active High"));
      regByte |= (1U << MCP23017_BIT_INTPOL); //write 1
    }

    //-----------------------------------------------------------------------------------//
    //reuse regByte since we need to keep previous modifications

    if (mirror == MCP23017_INT_MIRROR) {
      debugPort.println (F("Also mirror interrupt output"));
      regByte |= (1U << MCP23017_BIT_MIRROR); //write 1
    }
    else {
      regByte &= (~(1U << MCP23017_BIT_MIRROR));  //write 0
    }

    //-----------------------------------------------------------------------------------//

    debugPort.println (F("Writing IOCON"));
    //now write to device
    response = write (MCP23017_REG_IOCON, regByte, false); //write single byte
    debugPort.println (F("Success\n"));

    //save to register bank
    if (response == MCP23017_RESP_OK) {
      regBank [MCP23017_REG_IOCON] = regByte;
    }

    //-----------------------------------------------------------------------------------//
    
    if (attachHostInterrupt() == MCP23017_RESP_OK) {
      debugPort.println (F("Host MCU interrupt attach success\n"));
      isIntConfigured = true;
    }
    else {
      debugPort.println (F("Host MCU interrupt attach failed\n"));
      isIntConfigured = false;
      return MCP23017_ERROR_OF; //operation fail
    }

    return response;
  }

  return MCP23017_ERROR_OOR;
}

//===================================================================================//
//mode can be CHANGE (1), FALLING (2), RISING (3), LOW (4) or HIGH (5)
//mode = 0 is invalid state
//note that inverting the input polarity of pins/ports will also invert the interrupts
//means an actual falling edge signal will be treated as rising edge interrupt

int CSE_MCP23017:: attachInterrupt (uint8_t pin, ioeCallback_t isr, uint8_t mode) {
  if ((pin < MCP23017_PINCOUNT) && (mode <= MCP23017_INTERRUPT_COUNT)) {
    debugPort.print (F("Attaching interrupt to ioe pin "));
    debugPort.println (pin);

    if (readPinMode (pin) == OUTPUT) {
      debugPort.println (F("Pin is not configured as Input. Interrupts work only on Input pins.\n"));
      debugPort.print (F("Pin mode is "));
      debugPort.println (readPinMode(pin));
      return MCP23017_ERROR_OF;
    }

    if (isIntConfigured) { //check if interrupt is configured
      uint8_t regByte = 0;
      uint8_t response = 0;

      //read registers
      //interrupt enable
      debugPort.println (F("Reading registers"));
      regBank [MCP23017_REG_IODIRA] = read (MCP23017_REG_GPINTENA, false);
      regBank [MCP23017_REG_IODIRB] = read (MCP23017_REG_GPINTENB, false);

      regBank [MCP23017_REG_GPINTENA] = read (MCP23017_REG_GPINTENA, false);
      regBank [MCP23017_REG_GPINTENB] = read (MCP23017_REG_GPINTENB, false);
      
      //default compare value
      regBank [MCP23017_REG_DEFVALA] = read (MCP23017_REG_DEFVALA, false);
      regBank [MCP23017_REG_DEFVALB] = read (MCP23017_REG_DEFVALB, false);
      
      //interrupt control
      regBank [MCP23017_REG_INTCONA] = read (MCP23017_REG_INTCONA, false);
      regBank [MCP23017_REG_INTCONB] = read (MCP23017_REG_INTCONB, false);
      debugPort.println (F("Success"));

      //-----------------------------------------------------------------------------------//

      //-----------------------------------------------------------------------------------//
      //needs to make INTCON bit 0 and value in DEFVAL doesn't matter now
      
      if (mode == MCP23017_INT_CHANGE) {
        debugPort.println (F("Setting int mode to CHANGE"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //set 0
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;  //if success, save the value

          isrPtrList [pin] = isr; //save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; //save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //-----------------------------------------------------------------------------------//
      //needs to set INTCON bit to 1 and DEFVAL bit to 0
      //interrupt occurs at the time of opposite state
      //if DEFVAL is 0, a 0 -> 1 (rising) transition will cause an interrupt
      
      else if (mode == MCP23017_INT_RISING) {
        debugPort.println (F("Setting int mode to RISING"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Setting DEFVAL register"));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //set 0
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; //save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; //save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //-----------------------------------------------------------------------------------//
      //needs to set INTCON bit to 1 and DEFVAL bit to 1
      //interrupt occurs at the time of opposite state
      //if DEFVAL is 1, a 1 -> 0 (falling) transition will cause an interrupt
      
      else if (mode == MCP23017_INT_FALLING) {
        debugPort.println (F("Setting int mode to FALLING"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; //save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; //save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          debugPort.println (F("Interrupt configured for FALLING"));
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //-----------------------------------------------------------------------------------//
      //needs to set INTCON bit to 1 and DEFVAL bit to 1
      //state checking is same as falling edge interrupt
      
      else if (mode == MCP23017_INT_LOW) {
        debugPort.println (F("Setting int mode to LOW"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF; //write failure
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; //save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; //save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //-----------------------------------------------------------------------------------//
      //needs to set INTCON bit to 1 and DEFVAL bit to 0
      //state checking is same as rising edge interrupt
      
      else if (mode == MCP23017_INT_HIGH) {
        debugPort.println (F("Setting int mode to HIGH"));
        regByte = regBank [MCP23017_REG_INTCONA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
        response = write ((MCP23017_REG_INTCONA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_INTCONA + (pin >> 3)] = regByte;
        }
        else {
          return MCP23017_ERROR_WF;
        }

        debugPort.println (F("Writing DEFVAL register."));
        regByte = regBank [MCP23017_REG_DEFVALA + (pin >> 3)] & (~(0x1U << (pin & 0x7U))); //set 0
        response = write ((MCP23017_REG_DEFVALA + (pin >> 3)), regByte, false); //write single byte

        if (response == MCP23017_RESP_OK) {
          debugPort.println (F("Success"));
          regBank [MCP23017_REG_DEFVALA + (pin >> 3)] = regByte;
          
          isrPtrList [pin] = isr; //save the isr for that isrSupervisor can call this function
          isrModeList [pin] = mode; //save the interrupt mode for each pins so that isrSupervisor can check if the conditions are met
          
          // return MCP23017_RESP_OK;
        }
        else {
          return MCP23017_ERROR_WF;
        }
      }

      //-----------------------------------------------------------------------------------//
      //invalid mode

      else {
        debugPort.println (F("MCP23017 : Wrong interrupt mode (0). Failed to attach interrupt."));
      }

      //-----------------------------------------------------------------------------------//

      debugPort.println (F("Enabling Interrupt on Change"));
      //set GPINTEN to 1 enable the interrupt on change for each pin
      regByte = regBank [MCP23017_REG_GPINTENA + (pin >> 3)] | (0x1U << (pin & 0x7U));  //set 1
      response = write ((MCP23017_REG_GPINTENA + (pin >> 3)), regByte, false); //write single byte

      //save the value
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

//===================================================================================//
//attach the host MCU interrupt to detect interrupt output from the IO expander
//use configInterrupt to configure the pins and type

uint8_t CSE_MCP23017:: attachHostInterrupt() {
  debugPort.println (F("Attaching host MCU interrupt"));
  //active low means the signal will be a falling edge
  if (intOutType == MCP23017_ACTIVE_LOW) {
    debugPort.println (F("Output is Active Low"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      delay (100);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeId], FALLING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { //B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      delay (100);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeId], FALLING);
      debugPort.println (F("Success"));
    }
  }

  //active high means the signal will be a rising edge
  else if (intOutType == MCP23017_ACTIVE_HIGH) {
    debugPort.println (F("Output is Active High"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeId], RISING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { //B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeId], RISING);
      debugPort.println (F("Success"));
    }
  }

  //open drain means the interrupt out from MCP23017 will be either low
  //or high Z. if you choose to open drain, you have to either use the 
  //internal pull-up of your MCU or add an external pull-up. that means
  //the circuit will be equivalent to that of falling edge detection
  else if (intOutType == MCP23017_OPENDRAIN) {
    debugPort.println (F("Output is Open Drain"));
    if (attachPinA != -1) {
      debugPort.println (F("Attaching ISR to pin A"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      ::attachInterrupt (attachPinA, hostCallbackList [ioeId], FALLING);
      debugPort.println (F("Success"));
    }

    if (attachPinB != -1) { //B could be negative
      debugPort.println (F("Attaching ISR to pin B"));
      debugPort.print (F("ioeId is "));
      debugPort.println (ioeId);
      ::attachInterrupt (attachPinB, hostCallbackList [ioeId], FALLING);
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

//===================================================================================//

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

//===================================================================================//

bool CSE_MCP23017:: interruptPending() {
  return interruptActive;
}

//===================================================================================//
//this is the supervisor function that manages all ISRs.
//when an interrupt is registered by the MCU, the isrSupervisor should determine
//which pin of MCP23017 caused it and invoke the associated ISR set by the user.
//disable host MCU interrupts
//then read all the associated registers
//INTF, GPINTEN, DEFVAL, INTCON, and INTCAP in order.
//determine which pin caused the interrupt,
//verify the interrupt attached to the pin,
//call the ISR
//on return, re-attach the MCU interrups

void CSE_MCP23017:: isrSupervisor() {
  if (isIntConfigured == true) { //not necessary though
    //detach interrupts
    if (attachPinA != -1) {
      ::detachInterrupt (attachPinA);
    }
    
    if (attachPinB != -1) {
      ::detachInterrupt (attachPinB);
    }
    // debugPort.println (F("Detaching host interrupts"));
  }

  //-----------------------------------------------------------------------------------//
  //read registers

  //interrupt flag determines which pin caused the interrupt
  //this has to be read before disabling the IOE interrupts
  regBank [MCP23017_REG_INTFA] = read (MCP23017_REG_INTFA, false);
  regBank [MCP23017_REG_INTFB] = read (MCP23017_REG_INTFB, false);

  //interrupt enable
  regBank [MCP23017_REG_GPINTENA] = read (MCP23017_REG_GPINTENA, false);
  regBank [MCP23017_REG_GPINTENB] = read (MCP23017_REG_GPINTENB, false);

  write (MCP23017_REG_GPINTENA, 0);
  write (MCP23017_REG_GPINTENB, 0);

  //IOC control
  // regBank [MCP23017_REG_INTCONA] = read (MCP23017_REG_INTCONA, false);
  // regBank [MCP23017_REG_INTCONB] = read (MCP23017_REG_INTCONB, false);

  // write (MCP23017_REG_INTCONA, 0);
  // write (MCP23017_REG_INTCONB, 0);

  debugPort.println (F("ISR Supervisor invoked"));
  debugPort.println (F("Disabling IOE interrupts"));

  if ((!readError()) || (!writeError())) {
    debugPort.println (F("Success"));
  } else {
    debugPort.println (F("Failed"));
  }

  debugPort.println (F("Reading registers"));
  
  // //default compare value
  // regBank [MCP23017_REG_DEFVALA] = read (MCP23017_REG_DEFVALA, false);
  // regBank [MCP23017_REG_DEFVALB] = read (MCP23017_REG_DEFVALB, false);

  //interrupt capture
  regBank [MCP23017_REG_INTCAPA] = read (MCP23017_REG_INTCAPA, false);
  regBank [MCP23017_REG_INTCAPB] = read (MCP23017_REG_INTCAPB, false);

  if (!readError()) {
    debugPort.println (F("Success"));
  } else {
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

  //-----------------------------------------------------------------------------------//

  intPin = -1;
  intPinState = -1;
  intPinCapState = -1;

  //we have two bytes to check
  //we expect only one bit of either the registers to be a 1
  //to find where the bit 1 is, Rsh the bytes 1 bit at a time,
  //and compare it with 0x1. if it is 1, the final position
  // index will be the pin location
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

  //-----------------------------------------------------------------------------------//

  if (intPin == -1) { //if the pins couldn't be determined
    debugPort.println (F("MCP23017 Error : Unable to determine the pin interrupt occured at\n"));
    // return MCP23017_ERROR_UDP;
    // return;
  }
  else {
    lastIntPin = intPin;
    //read the pin state when interrupt occured
    // intPinCapState = regBank[MCP23017_REG_INTCAPA + (intPin >> 3)] & (0x1U << (intPin & 0x7));
    intPinCapState = (regBank [MCP23017_REG_INTCAPA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U;
      
    //-----------------------------------------------------------------------------------//
    //if the interrupt is set for LOW state, we have to read the port register every time
    //after the ISR is finished and call it again if the state persists
    
    if (isrModeList [intPin] == MCP23017_INT_LOW) {
      if (intPinCapState == 0) { //if the bit pos is 0
        bool statePersist = false;

        do {
          isrPtrList [intPin] (intPin);  //call the ISR attached to the pin

          read (regBank [MCP23017_REG_GPIOA + (intPin >> 3)]);  //read just the associated reg only to save time
          intPinState = regBank [MCP23017_REG_GPIOA + (intPin >> 3)] & (0x1U << (intPin & 0x7));

          if (intPinState == 0) { //if the state persists
            statePersist = true;
          }
        } while (statePersist == true); //repeat
      }
    }

    //-----------------------------------------------------------------------------------//
    //if the interrupt is set for HIGH state, we have to read the port register every time
    //after the ISR is finished and call it again if the state persists
    
    else if (isrModeList [intPin] == MCP23017_INT_HIGH) {
      //the bit that is 1 may appear anywhere on the byte
      //so we just need to check if the result if > 0
      //instead of shifting 1U to arbitrary left, the reg value can
      //itself be shifted to right as,
      //(regBank[MCP23017_REG_INTCAPA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U
      //this is only valid for reading the bit pos in a reg
      if (intPinCapState == 1) { //if the bit pos is 1
        bool statePersist = false;
        
        do {
          isrPtrList [intPin] (intPin);  //call the ISR attached to the pin

          read (regBank [MCP23017_REG_GPIOA + (intPin >> 3)]);  //read just the associated reg only to save time
          intPinState = (regBank [MCP23017_REG_GPIOA + (intPin >> 3)] >> (intPin & 0x7)) & 0x1U;
          
          if (intPinState == 1) { //if the state persists
            statePersist = true;
          }
        } while (statePersist == true); //repeat
      }
    }

    //-----------------------------------------------------------------------------------//
    //if the interrupt is set for CHANGE of state, then we do not need check any registers
    //because the interrupt could have occured when a state of change occured
    //and it occurs only once
    
    else if (isrModeList [intPin] == MCP23017_INT_CHANGE) {
      isrPtrList [intPin] (intPin);  //call the ISR attached to the pin
    }

    //-----------------------------------------------------------------------------------//
    //this is simlar to LOW state interrupt excpet the ISR is called only once

    else if (isrModeList [intPin] == MCP23017_INT_FALLING) {
      if (intPinCapState == 0) { //if the bit pos is 0, that means the pin state changed from HIGH -> LOW
        isrPtrList [intPin] (intPin);  //call the ISR attached to the pin
      }
    }

    //-----------------------------------------------------------------------------------//
    //this is simlar to HIGH state interrupt excpet the ISR is called only once

    else if (isrModeList [intPin] == MCP23017_INT_RISING) {
      if (intPinCapState == 1) { //if the bit pos is 0, that means the pin state changed from HIGH -> LOW
        isrPtrList [intPin] (intPin);  //call the ISR attached to the pin
      }
    }

    //-----------------------------------------------------------------------------------//

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
//reverses ASCII formatted binary number string

void reverseString (char *sourceString) { // modify the source string
  uint8_t temp;  // copy the char
  int32_t arraySize = strlen (sourceString);

  for (int i = 0; i < (arraySize / 2); i++) {
    temp = sourceString [i]; // copy the char
    sourceString [i] = sourceString [(arraySize - 1) - i]; // switch the digits from right to left
    sourceString [(arraySize - 1) - i] = temp; // copy the left half char to right
  }
}

//===============================================================================//
//converts a decimal number to its binary representation string
//input is a number and the minimum length of the string
//if the original binary of the number is less than the width,
//0s will be padded

String toBinary (uint64_t number, uint16_t width = 0) {
  uint8_t i = 0;
  uint8_t j = 0;
  char binaryBuffer [65] = {0}; // 65 bits - why odd number is because we can split the binary string half

  while ((number > 0) || (j < width)) { // loop until number becomes 0
    sprintf (&binaryBuffer [i++], "%u", (number & 1)); // AND each LSB with 1, don't use %llu
    j++;
    number >>= 1; // shift the number to right
  }

  reverseString (binaryBuffer); // reverse the string

  // debugPort.print(F("\nbinaryBuffer : "));
  // for (int k=0; k<j; k++) {
  //   debugPort.print(binaryBuffer[k]);
  // }

  return (String (binaryBuffer));
}

//===============================================================================//


