#include "XDB401_Library.h"

// Constructor
XDB401::XDB401()
{
}

// Initialize the XDB401 sensor
bool XDB401::begin(TwoWire &wirePort, bool reset)
{
  _i2cPort = &wirePort;

  if (!isConnected())
  {
    if (!isConnected()) // Retry once
      return false;
  }

  if (reset)
  {
    if (!sendCommand()) // Start initial acquisition
      return false;
  }

  return true;
}

// Check if sensor acknowledges at I2C address
bool XDB401::isConnected()
{
  _i2cPort->beginTransmission(_address);
  return (_i2cPort->endTransmission() == 0);
}

// Send acquisition command (0x0A to register 0x30)
bool XDB401::sendCommand()
{
  return setRegister(XDB401_STATUS, 0x0A);
}

// Check if data acquisition is complete (SCO bit = 0)
bool XDB401::isDataReady()
{
  uint8_t status = getRegister(XDB401_STATUS);
  return (status & (1 << XDB401_SCO_BIT)) == 0;
}

// Tare the sensor by averaging numReadings ADC values
bool XDB401::tare(uint8_t numReadings)
{
  if (numReadings == 0)
    return false;

  long sum = 0;
  uint8_t validReadings = 0;

  for (uint8_t i = 0; i < numReadings; i++)
  {
    if (!sendCommand())
      continue;

    while (!isDataReady())
    {
      delay(10);
    }

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(XDB401_DATA_P0);
    _i2cPort->endTransmission();

    _i2cPort->requestFrom(_address, (uint8_t)3);
    if (_i2cPort->available() >= 3)
    {
      byte x = _i2cPort->read();
      byte y = _i2cPort->read();
      byte z = _i2cPort->read();
      long m = ((long)x << 16) | ((long)y << 8) | z;
      sum += m;
      validReadings++;
    }
    delay(10); // Small delay between readings
  }

  if (validReadings == 0)
    return false;

  _zeroOffset = sum / validReadings;
  return true;
}

// Set the calibration factor for PSI conversion
void XDB401::setCalibrationFactor(float factor)
{
  _calibrationFactor = factor;
}

// Set reading interval in readings per second
void XDB401::setReadingsPerSecond(float rps)
{
  if (rps > 0)
    _readIntervalMs = 1000.0 / rps;
  else
    _readIntervalMs = 1000.0; // Default to 1 Hz if invalid
}

// Read pressure in PSI
float XDB401::readPressure()
{
  if (!sendCommand())
    return -1.0;

  while (!isDataReady())
  {
    delay(10);
  }

  _i2cPort->beginTransmission(_address);
  _i2cPort->write(XDB401_DATA_P0);
  _i2cPort->endTransmission();

  _i2cPort->requestFrom(_address, (uint8_t)3);
  if (_i2cPort->available() >= 3)
  {
    byte x = _i2cPort->read();
    byte y = _i2cPort->read();
    byte z = _i2cPort->read();

    long m = ((long)x << 16) | ((long)y << 8) | z;
    long zeroed_m = m - _zeroOffset;
    float norm_p = zeroed_m / 8388608.0;
    if (zeroed_m > 8388608L) norm_p -= 2.0;
    float pressure_psi = norm_p * _calibrationFactor;
    return pressure_psi;
  }

  return -1.0; // Return -1.0 if reading fails
}

// Average multiple pressure readings
float XDB401::calculateAverage(uint8_t numReadings)
{
  if (numReadings == 0)
    return -1.0;

  float sum = 0.0;
  uint8_t validReadings = 0;

  for (uint8_t i = 0; i < numReadings; i++)
  {
    float pressure = readPressure();
    if (pressure >= -1000.0 && pressure <= 1000.0)
    {
      sum += pressure;
      validReadings++;
    }
    delay(_readIntervalMs / numReadings); // Distribute delay
  }

  if (validReadings == 0)
    return -1.0;

  return sum / validReadings;
}

// Write to a register
bool XDB401::setRegister(uint8_t reg, uint8_t value)
{
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  return (_i2cPort->endTransmission() == 0);
}

// Read from a register
uint8_t XDB401::getRegister(uint8_t reg)
{
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission();

  _i2cPort->requestFrom(_address, (uint8_t)1);
  if (_i2cPort->available())
    return _i2cPort->read();
  return 0;
}