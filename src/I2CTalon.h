/******************************************************************************
AuxTalon
Interface for Aux Talon
Bobby Schulz @ GEMS Sensing
5/18/2022
https://github.com/gemsiot/Driver_-_Talon-Auxiliary

Allows control of all elements of the Aux Talon, including IO interfacing and self diagnostics 

0.0.0

///////////////////////////////////////////////////////////////////FILL QUOTE////////////////////////////////////////////////////////////////////////////////

Distributed as-is; no warranty is given.
******************************************************************************/
/**
 * @file AuxTalon.h
 *
 * @mainpage Auxiliary Talon
 *
 * @section description Description
 * An interface to exist between the Auxiliary Talon and the Kestrel data logger
 *
 * @section circuit Circuit
 * - Red LED connected to pin D2.
 * - Momentary push button connected to pin D3.
 *
 * @section libraries Libraries
 * - Arduino_LSM6DS3 (https://github.com/arduino-libraries/Arduino_LSM6DS3)
 *   - Interacts with on-board IMU.
 *
 * @section notes Notes
 * - Comments are Doxygen compatible.
 *
 * @section todo TODO
 * - Don't use Doxygen style formatting inside the body of a function.
 *
 * @section author Author
 * - Created by Bobby Schulz on 5/20/2022
 *
 * Copyright (c) 2020 Woolsey Workshop.  All rights reserved.
 */

#ifndef I2CTalon_h
#define I2CTalon_h

#include <stdio.h>
// #include "Arduino.h"
#include <Particle.h>
#include <Wire.h>
#include "PCAL9535A.h"
#include <MCP3421.h>
#include <SparkFun_PCA9536_Arduino_Library.h>
#include <Sensor.h>
#include <Talon.h>
#include <GlobalPins.h>
// #include "Adafruit_ADS1015.h"

// enum GroupMode{
//  Dim = 0,
//  Blink = 1,
// };

// enum OutputMode{
//  OpenDrain = 0,
//  TotemPole = 1,
// };

// namespace OutputType 
// {
//     constexpr uint8_t OPEN_DRAIN = 1;
//     constexpr uint8_t PUSH_PULL = 0;
// };

//! Pins for IO expander Alpha
/*! Variable names corespond directly to signal names in Eagle CAD. */
// namespace pinsAlpha 
// {
//   constexpr uint8_t EN1 = 0;
//   constexpr uint8_t EN2 = 1;
//   constexpr uint8_t EN3 = 2;
//   constexpr uint8_t EN4 = 3;
//   constexpr uint8_t FAULT1 = 8;
//   constexpr uint8_t FAULT2 = 9;
//   constexpr uint8_t FAULT3 = 10;
//   constexpr uint8_t FAULT4 = 11;
//   constexpr uint8_t DATA_EN1 = 4;
//   constexpr uint8_t DATA_EN2 = 5;
//   constexpr uint8_t DATA_EN3 = 6;
//   constexpr uint8_t DATA_EN4 = 7;
//   constexpr uint8_t POS_DETECT = 12;
//   constexpr uint8_t SENSE_EN = 13;
//   constexpr uint8_t LOOPBACK_EN = 14;
// };

namespace pinsSense
{
  constexpr uint8_t MUX_EN = 3;
  constexpr uint8_t MUX_SEL0 = 0;
  constexpr uint8_t MUX_SEL1 = 1;
  constexpr uint8_t MUX_SEL2 = 2;
};

// namespace pinsBeta 
// {
//   constexpr uint8_t OUT1 = 0;
//   constexpr uint8_t OUT2 = 1;
//   constexpr uint8_t OUT3 = 2;
//   constexpr uint8_t COUNT_EN1 = 3;
//   constexpr uint8_t COUNT_EN2 = 4;
//   constexpr uint8_t COUNT_EN3 = 5;
//   constexpr uint8_t LOAD = 6;
//   constexpr uint8_t OVF1 = 7;
//   constexpr uint8_t OVF2 = 8;
//   constexpr uint8_t OVF3 = 9;
//   constexpr uint8_t D1_SENSE = 10;
//   constexpr uint8_t D2_SENSE = 11;
//   constexpr uint8_t D3_SENSE = 12;
//   constexpr uint8_t OD1 = 13;
//   constexpr uint8_t OD2 = 14;
//   constexpr uint8_t OD3 = 15;
// };

/**
 * @class AuxTalon
 * @brief Library for the Auxiliar Talon interfacing
 */
class I2CTalon: public Talon
{
  constexpr static  int DEAFULT_PORT = 3; ///<Use port 3 by default
  constexpr static  int DEFAULT_VERSION = 0x21; ///<Use hardware version v2.1 by default
  constexpr static  int MAX_NUM_ERRORS = 10; ///<Maximum number of errors to log before overwriting previous errors in buffer
  const String FIRMWARE_VERSION = "1.4.0"; //FIX! Read from system??
  
  enum pinsAlpha 
  {
    // constexpr uint8_t EN1 = 0;
    // constexpr uint8_t EN2 = 1;
    // constexpr uint8_t EN3 = 2;
    // constexpr uint8_t EN4 = 3;
    // constexpr uint8_t FAULT1 = 8;
    // constexpr uint8_t FAULT2 = 9;
    // constexpr uint8_t FAULT3 = 10;
    // constexpr uint8_t FAULT4 = 11;
    // constexpr uint8_t DATA_EN1 = 4;
    // constexpr uint8_t DATA_EN2 = 5;
    // constexpr uint8_t DATA_EN3 = 6;
    // constexpr uint8_t DATA_EN4 = 7;
    // constexpr uint8_t POS_DETECT = 12;
    // constexpr uint8_t SENSE_EN = 13;
    // constexpr uint8_t LOOPBACK_EN = 14;
    EN1 = 0,
    EN2 = 1,
    EN3 = 2,
    EN4 = 3,
    FAULT1 = 8,
    FAULT2 = 9,
    FAULT3 = 10,
    FAULT4 = 11,
    DATA_EN1 = 4,
    DATA_EN2 = 5,
    DATA_EN3 = 6,
    DATA_EN4 = 7,
    POS_DETECT = 12,
    SENSE_EN = 13,
    LOOPBACK_EN = 14
  };
  ////////////// ERROR CODES ///////////////
  // const uint32_t ADC_I2C_ERROR = 0xFF00; //FIX!!! (Low 3 bits are returned error)
  // const uint32_t ADC_TIMEOUT_ERROR = 0xFF10; //FIX!!!
  // const uint32_t COUNTER_OVERFLOW = 0xFF20; //FIX!!! (low 2 bits are which port)
  // const uint32_t TIME_DELTA_EXCEEDED = 0xFF30; //FIX!!! 
  // const uint32_t TIME_BAD = 0xFF40; //FIX!
  // const uint32_t DEVICE_RESET = 0xFF50; //FIX!
  // const uint32_t POWER_FAULT = 0xFF60; //FIX! (low 2 bits are which port)
  // const uint32_t POWER_FAULT_PERSISTENT = 0xFF70; //FIX! (low 2 bits are which port)
  // const uint32_t BUS_DISAGREE = 0xFF80; //FIX! (low 2 bits are which port)
  // const uint32_t BUS_OUTOFRANGE = 0xFF90; //FIX! (low 2 bits are which port, 3 = 5V rail)
  const uint32_t IO_INIT_ERROR = 0xFFA0; //FIX! (Low 3 bits are returned error)
  // const uint32_t ADC_INIT_ERROR = 0xFFB0; //FIX! (Low 3 bits are returned error)
  // const uint32_t INPUT_BUF_ERROR = 0xFFC0; //FIX! (Low 2 bits are which port, 3rd bit is Dx vs ODx input)
  // const uint32_t COUNTER_INCREMENT_ERROR = 0xFFD0; //FIX! (Low 2 bits are which port)
  // const uint32_t COUNTER_CLEAR_ERROR = 0xFFE0; //FIX!
  const uint32_t EEPROM_I2C_ERROR = 0xFFF0; //FIX! (Low 3 bits are returned error)
  const uint32_t SENSE_ADC_INIT_FAIL = 0x10060000; 
  const uint32_t SENSOR_PORT_RANGE_ERROR = 0x90010100; 
  // const uint32_t SENSOR_POWER_FAIL = 0x20010000; //(low 2 bits are which port)
  // const uint32_t SENSOR_POWER_FAIL_PERSISTENT = 0x20010100; //(low 2 bits are which port)
  const uint32_t I2C_OB_ISO_FAIL = 0x0F00; //FIX! 
  const uint32_t I2C_PORT_FAIL = 0x0FE0; //FIX! 
  // const float MAX_DISAGREE = 0.1; //If bus is different from expected by more than 10%, throw error

  

  public:

    /**
     * @brief Instantiate the Talon, defaults to using pre-specified port and hardware version
     * * @param[in] talonPort: coresponds with the number of the port on the Kestrel logger that the Talon is plugged into
     * * @param[in] version: Describes the hardware version of the Talon to be used 
     * @details Serves only to copy set global values of port and version
     */    
    I2CTalon(uint8_t talonPort_ = DEAFULT_PORT, uint8_t version = DEFAULT_VERSION); 
    /**
     * @brief Setsup the Talon for operation and sets all configurations
     * @details Configures IO expander pins, runs a level 2 Self Diagnostic, and resets the counters 
     */ 
    String begin(time_t time, bool &criticalFault, bool &fault);
    int sleep(bool State);

    // uint16_t counts[3] = {0}; //Used to store the register values after a read 
    // float rates[3] = {0}; //Used to store the calculated rate values from registers
    // bool overflow[3] = {false}; //Used to store if any of the registers have overflowed in the previous period
    // bool faults[3] = {false}; //Used to store if any of the ports have had a power fault 
    // float analogValsAvg[3] = {0}; //Used to store the averaged analog reading outputs
    // float analogVals[3] = {0}; //Used to store the instantanious analog values
    // bool portVoltageSettings[3] = {0}; //Used to store the configurations of each port (0 = 3v3, 1 = 5v0) - updated at each call of level 4 or greater
    
    // String errorTags[MAX_NUM_ERRORS]
    
    // String getData(time_t time);
    bool hasReset();
    int restart();
    String selfDiagnostic(uint8_t diagnosticLevel = 4, time_t time = 0); //Default to just level 4 diagnostic, default to time = 0
    // int sleepMode(uint8_t mode) //DEFINE!
    // int reportErrors(uint32_t *errors, size_t length);
    String getErrors();
    String getMetadata();
    // uint8_t totalErrors();
    bool ovfErrors();
    // uint8_t getPort();
    // void setTalonPort(uint8_t port);
    // bool isTalon() {
    //   return true;
    // };
    int enableData(uint8_t port, bool state);
    int enablePower(uint8_t port, bool state);
    // int disableDataAll();
    // int disablePowerAll();
    // uint8_t getTalonPort() {
    //   return talonPort + 1;
    // }
    bool isPresent();
    uint8_t getNumPorts() {
      return numPorts;
    }
    // const uint8_t sensorInterface = BusType::I2C;

  private:
    const uint8_t numPorts = 4; 
    PCAL9535A ioAlpha; //ADR = 0x22
    MCP3421 adcSense;
    PCA9536 ioSense;
    uint8_t expectedI2CVals[7] = {0x00, 0x22, 0x30, 0x41, 0x50, 0x58, 0x6B};
    const uint16_t maxTalonCurrent = 13104; //CSA reading for 2A output
    const uint16_t maxPortCurrent = 3276; //CSA reading for 500mA output
    const float voltageDiv = 2; ///<Program voltage divider
    const float currentDiv = 0.243902439; ///<82mOhm, 50V/V Amp
    // int throwError(uint32_t error);
    void setPinDefaults();


    // bool hasReset(); 

    bool testOvercurrent(uint16_t baseCurrent);
    uint16_t getBaseCurrent();
    uint16_t getCurrent();



    // static time_t readTime = 0;
    // bool initDone = false; //Used to keep track if the initaliztion has run - used by hasReset() 
    
    bool faults[4] = {false, false, false, false}; //Used to store if any of the ports have had a power fault
    // uint32_t errors[MAX_NUM_ERRORS] = {0};
    // uint8_t numErrors = 0; //Used to track the index of errors array
    // bool errorOverwrite = false; //Used to track if errors have been overwritten in time since last report
    // bool timeBaseGood = false; //Used to keep track of the valitity of the current timebase
    // uint8_t talonPort = 255; //Used to keep track of which port the Talon is connected to on Kestrel
    // uint32_t portErrorCode = 0; //Used to easily OR with error codes to add the Talon port
    uint8_t version = 0; //FIX! This should be read from EEPROM in future 
};

#endif