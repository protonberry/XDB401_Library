#ifndef XDB401_Library_h
#define XDB401_Library_h

#include "Arduino.h"
#include <Wire.h>

// Register Map for XDB401 Pressure Sensor
typedef enum
{
  XDB401_STATUS = 0x30,  // Command and status register
  XDB401_DATA_P0 = 0x06, // Pressure high byte
  XDB401_DATA_P1 = 0x07, // Pressure middle byte
  XDB401_DATA_P2 = 0x08, // Pressure low byte
} XDB401_Registers;

// Status bit for acquisition completion
typedef enum
{
  XDB401_SCO_BIT = 3, // Bit 3 indicates acquisition complete (0 = done)
} XDB401_Status_Bits;

class XDB401
{
public:
  XDB401(); // Default constructor
  bool begin(TwoWire &wirePort = Wire, bool reset = true); // Initialize sensor
  bool isConnected(); // Check if sensor acknowledges at I2C address
  float readPressure(); // Read pressure in PSI
  bool tare(uint8_t numReadings = 1); // Zero the sensor, averaging numReadings
  void setCalibrationFactor(float factor); // Set calibration factor for PSI conversion
  float calculateAverage(uint8_t numReadings); // Average multiple pressure readings
  void setReadingsPerSecond(float rps); // Set reading interval in readings per second

private:
  TwoWire *_i2cPort; // I2C port used
  const uint8_t _address = 0x7F; // Fixed I2C address for XDB401
  long _zeroOffset = 0; // Offset for taring (raw ADC value)
  float _calibrationFactor = 1.0; // Default calibration factor
  float _readIntervalMs = 1000.0; // Default 1 reading per second (1000ms)
  bool sendCommand(); // Send acquisition command (0x0A to 0x30)
  bool isDataReady(); // Check if acquisition is complete
  bool setRegister(uint8_t reg, uint8_t value); // Write to register
  uint8_t getRegister(uint8_t reg); // Read from register
};

#endif