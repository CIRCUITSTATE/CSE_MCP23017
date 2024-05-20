
//===================================================================================//

#ifndef CSE_MCP23017_H
#define CSE_MCP23017_H

#include <Arduino.h>
#include <Wire.h>
// #include <string>

//===================================================================================//

#define   debugPort   Serial

// Constants
#define   MCP23017_ADDRESS            0x20U  // 0b01000111
#define   MCP23017_REGADDR_MAX        0x15U  // Even though absolute or translated address can be greater than this
#define   MCP23017_PINCOUNT           0x10U  // Total GPIO pins of MCP23017
#define   MCP23017_PINMODES           0x3U   // No. of pin modes of GPIO pins
#define   MCP23017_PORTCOUNT          0x2U   // Port count of MCP23017
#define   MCP23017_INTERRUPT_COUNT    0x5U
#define   MCP23017_INT_MIRROR         0x1U   // To mirror INTA to INTB
#define   MCP23017_INT_NOMIRROR       0x0U   // To not to mirror INTA to INTB
#define   MCP23017_MAX_OBJECT         0x6U   // Max no. of objects that support interrupt

// Pins
#define   MCP23017_GPA0               0U
#define   MCP23017_GPA1               1U
#define   MCP23017_GPA2               2U
#define   MCP23017_GPA3               3U
#define   MCP23017_GPA4               4U
#define   MCP23017_GPA5               5U
#define   MCP23017_GPA6               6U
#define   MCP23017_GPA7               7U
#define   MCP23017_GPB0               8U
#define   MCP23017_GPB1               9U
#define   MCP23017_GPB2               10U
#define   MCP23017_GPB3               11U
#define   MCP23017_GPB4               12U
#define   MCP23017_GPB5               13U
#define   MCP23017_GPB6               14U
#define   MCP23017_GPB7               15U

// Error Codes
#define   MCP23017_ERROR_OOR          0x64U  // Address out of range
#define   MCP23017_ERROR_WF           0x65U  // Device write fail
#define   MCP23017_ERROR_PAE          0x66U  // Pin assignment error
#define   MCP23017_ERROR_UDP          0x67U  // Unable to determine pin
#define   MCP23017_ERROR_OF           0x68U  // Operation fail

// Response Codes
#define   MCP23017_RESP_OK            0x0

// Location offsets of each register in regArray
#define   MCP23017_REG_IODIRA         0x0U   // IO direction A, RW
#define   MCP23017_REG_IODIRB         0x1U   // IO direction B, RW
#define   MCP23017_REG_IPOLA          0x2U   // Input polarity A, RW
#define   MCP23017_REG_IPOLB          0x3U   // Input polarity B, RW
#define   MCP23017_REG_GPINTENA       0x4U   // Interrupt-on-change enable A, RW
#define   MCP23017_REG_GPINTENB       0x5U   // Interrupt-on-change enable B, RW
#define   MCP23017_REG_DEFVALA        0x6U   // Default value for compare A, RW
#define   MCP23017_REG_DEFVALB        0x7U   // Default value for compare B, RW
#define   MCP23017_REG_INTCONA        0x8U   // Interrupt type A, RW
#define   MCP23017_REG_INTCONB        0x9U   // Interrupt type B, RW
#define   MCP23017_REG_IOCON          0xAU   // IO port control, RW
#define   MCP23017_REG_IOCON_         0xBU   // IO port control, RW
#define   MCP23017_REG_GPPUA          0xCU   // Pull-up resistor enable A, RW
#define   MCP23017_REG_GPPUB          0xDU   // Pull-up resistor enable B, RW
#define   MCP23017_REG_INTFA          0xEU   // Interrupt flag A, R
#define   MCP23017_REG_INTFB          0xFU   // Interrupt flag B, R
#define   MCP23017_REG_INTCAPA        0x10U  // Interrupt capture A, R
#define   MCP23017_REG_INTCAPB        0x11U  // Interrupt capture B, R
#define   MCP23017_REG_GPIOA          0x12U  // Port register A, RW
#define   MCP23017_REG_GPIOB          0x13U  // Port register B, RW
#define   MCP23017_REG_OLATA          0x14U  // Output latch A, RW
#define   MCP23017_REG_OLATB          0x15U  // Output latch B, RW

// Bit Positions
#define   MCP23017_BIT_BANK           7U
#define   MCP23017_BIT_MIRROR         6U
#define   MCP23017_BIT_SEQOP          5U
#define   MCP23017_BIT_DISSLW         4U
#define   MCP23017_BIT_HAEN           3U
#define   MCP23017_BIT_ODR            2U
#define   MCP23017_BIT_INTPOL         1U

// Signals
#define   MCP23017_HIGH               HIGH
#define   MCP23017_LOW                LOW

// Signal Types
#define   MCP23017_ACTIVE_LOW         0U
#define   MCP23017_ACTIVE_HIGH        1U
#define   MCP23017_OPENDRAIN          2U

// Interrupt Types
#define   MCP23017_INT_LOW            4U  // Interrupt on LOW
#define   MCP23017_INT_HIGH           5U  // Interrupt on HIGH
#define   MCP23017_INT_CHANGE         1U  // Interrupt on change
#define   MCP23017_INT_FALLING        2U  // Interrupt on falling edge
#define   MCP23017_INT_RISING         3U  // Interrupt on rising edge

//===================================================================================//
// Macro Functions

// Translates to sequential (group) address for both port A and B.
// All port A register addresses are even numbers, and only needs a single Rsh.
// All port B register addresses are odd numbers, and requires a single Rsh and addition of 16.
#define TRANSLATE(a) ((a >> 1) + (0x10U * (a >> 1)))

// #define READ_PIN_REGISTER(pin, reg, translate) (regBank[reg + (pin >> 3)] = read((reg + (pin >> 3)), translate))

//===================================================================================//
// Typedefs

typedef void (*hostCallback_t)(void);  // The type of callback the host MCU will make
typedef void (*ioeCallback_t)(int8_t);  // The type of callback the IO expander will make

//===================================================================================//
// Forward declarations

String toBinary (uint64_t number, uint16_t width);

//===================================================================================//

class CSE_MCP23017 {
  private:
    enum bankModes {
      PAIR,
      GROUP
    };
    
    uint8_t resetPin = 0; // GPIO where reset pin of IOE is connected
    uint8_t deviceAddress = 0;
    uint8_t i2cTxBuffer [22] = {0};  // I2C transmit buffer
    bankModes bankMode = PAIR; // 0 (false) = pair mode, 1 (true)= group mode
    uint8_t addressMode = 0; // 0 = sequential mode, 1 = byte mode (no address auto-increment)
    int8_t attachPinA = -1; // Interrupt attach pin A
    int8_t attachPinB = -1; // Interrupt attach pin B
    uint8_t intOutType = 0; // Interrupt output type
    bool isIntConfigured = false; // Is interrupt configured
    uint8_t ioeId = 0;  // IO expander object index

    ioeCallback_t isrPtrList [MCP23017_PINCOUNT] = {NULL};  // Array to hold interrupt function pointers
    uint8_t isrModeList [MCP23017_PINCOUNT] = {0};

    bool deviceReadError;
    bool deviceWriteError;

    uint8_t attachHostInterrupt();
    
  public:
    enum gpioPin {
      GPIOA0,
      GPIOA1,
      GPIOA2,
      GPIOA3,
      GPIOA4,
      GPIOA5,
      GPIOA6,
      GPIOA7,
      GPIOB0,
      GPIOB1,
      GPIOB2,
      GPIOB3,
      GPIOB4,
      GPIOB5,
      GPIOB6,
      GPIOB7
    };
    
    uint8_t regBank [22] = {0};  //register content
    int8_t intPin;
    int8_t intPinState;
    int8_t intPinCapState;
    hostCallback_t callback;
    volatile bool interruptActive;
    volatile bool stateReverted;
    int8_t lastIntPin;
    
    CSE_MCP23017 (uint8_t resetPin, uint8_t address);
    ~CSE_MCP23017();
    void reset();
    uint8_t begin();
    uint8_t write (uint8_t regAddress, uint8_t *buffer, uint8_t bufferOffset, uint8_t length, bool translateAddress = false);
    uint8_t write (uint8_t regAddress, uint8_t byteOne, bool translateAddress = false);
    uint8_t write (bool translateAddress = false);
    uint8_t read (uint8_t regAddress, bool translateAddress = false);
    uint8_t readRegisters (bool translateAddress = false);
    uint8_t update (uint8_t regOffset, uint8_t *buffer, uint8_t bufferOffset, uint8_t length);
    uint8_t update (uint8_t regOffset, uint8_t byteOne, uint8_t byteTwo);
    bool writeError();
    void writeError(bool e);
    bool readError();
    void readError(bool e);
    bool printOperationStatus (bool input);
    uint8_t update (uint8_t regOffset, uint8_t byteOne);
    uint8_t pinMode (uint8_t pin, uint8_t mode);
    uint8_t portMode (uint8_t port, uint8_t mode);
    uint8_t digitalWrite (uint8_t pin, uint8_t value);
    uint8_t portWrite (uint8_t port, uint8_t value);
    uint8_t togglePort (uint8_t port);
    uint8_t togglePin (uint8_t pin);
    uint8_t digitalRead (uint8_t pin);
    uint8_t readPinMode (uint8_t pin);
    uint8_t readPinBit (uint8_t pin, uint8_t reg, bool translate = false);
    uint8_t portRead (uint8_t port);
    uint8_t setPinInputPolarity (uint8_t pin, uint8_t value);
    uint8_t setPortInputPolarity (uint8_t port, uint8_t value);
    uint8_t configInterrupt (int8_t attachPin, uint8_t outType, uint8_t mirror);
    uint8_t configInterrupt (int8_t attachPin1, int8_t attachPin2, uint8_t outType, uint8_t mirror);
    int attachInterrupt (uint8_t pin, ioeCallback_t isr, uint8_t mode);
    void isrSupervisor();
    void dispatchInterrupt();
    bool interruptPending();
};

#endif

//===================================================================================//

