// encoder.ino

#include "encoder.h"

float readDegreeAngle(TwoWire &wire)
{
  wire.beginTransmission(AS5600_ADDR);
  wire.write(ANGLE_ADDR);      // AS5600 register 0x0E for angle
  wire.endTransmission(false); // Don't end transmission for continuous sending

  wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes for angle data

  if (wire.available() < 2)
    return COM_FAIL;

  uint8_t highByte = wire.read();
  uint8_t lowByte = wire.read();

  // Convert to angle in degrees
  float degreeAngle = ((highByte << 8) | lowByte) * 0.0879;
  return degreeAngle;
}

bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition)
{
  /*
  description : set zero position
  @param &wire  (TwoWire) sensor
  @param zeroPosition  (unsigned int)  the zero position value (12 bits)
  @return (bool) success
  */

  // Check if the zeroPosition is within range (12-bit)
  if (zeroPosition >= pow(2, 12))
    return COM_FAIL;

  uint8_t highByte = zeroPosition >> 8;
  uint8_t lowByte = zeroPosition & 0xFF;

  wire.beginTransmission(AS5600_ADDR);
  wire.write(ZPOSH_ADDR); // AS5600 register 0x01 for high byte of ZPOS
  wire.write(highByte);   // Write high byte of zero position
  wire.write(ZPOSL_ADDR); // AS5600 register 0x02 for low byte of ZPOS
  wire.write(lowByte);    // Write low byte of zero position

  // Check if transmission was successful
  if (wire.endTransmission() == 0)
    return COM_SUCCESS;

  return COM_FAIL;
}

uint8_t readStatus(TwoWire &wire)
{
  /*
  description : Read status register and return a 5-bit array
  @param &wire  (TwoWire) sensor
  @returns (byte)5bit_status
          [ bit 0 - MD (Magnet Detected): magnet is detected,
          bit 1 - ML (Magnet Too Low): magnet too low,
          bit 2 - MH (Magnet Too High): magnet too high,
          bit 3 - CO (Cordic Overflow):Cordic overflow error occurred,
          bit 4 - No special error ]
  */

  uint8_t status;
  uint8_t result = 0; // Initialize result

  wire.beginTransmission(AS5600_ADDR);
  wire.write(0x0B); // STATUS register address
  wire.endTransmission();
  wire.requestFrom(AS5600_ADDR, 1); // Request 1 byte

  if (!wire.available())
    return result;

  // Read status register
  status = wire.read();

  // Extract bits from the status byte
  bool magnetDetected = status & 0x01;
  bool magnetLow = status & 0x02;
  bool magnetHigh = status & 0x04;
  bool cordicOverflow = status & 0x08;

  // Assign bits to the result
  result |= (magnetDetected << 0);
  result |= (magnetLow << 1);
  result |= (magnetHigh << 2);
  result |= (cordicOverflow << 3);

  // Set the 5th bit if no special error (magnetLow, magnetHigh, cordicOverflow) is detected
  if (!magnetLow && !magnetLow && !magnetLow)
    result |= (1 << 4); // Set bit 4 if no errors

  return result; // Return the 5-bit status representation
}

float readRPM(TwoWire &wire)
{
  /*
  @brief Read angular velocity using AS5600 sensor
  @param &wire TwoWire : reference to I2C bus instance
  @note 이 함수는 주기적으로 호출되어야함
  @return float : angular velocity in degrees per second, or COM_FAIL if fail
  */
  static float previousAngle = 0.0;
  static unsigned long previousMicros = 0;

  wire.beginTransmission(AS5600_ADDR);
  wire.write(ANGLE_ADDR);
  wire.endTransmission(false);
  wire.requestFrom(AS5600_ADDR, 2);

  if (wire.available() < 2)
    return COM_FAIL;

  byte highByte = wire.read();
  byte lowByte = wire.read();
  float currentRawAngle = (highByte << 8) | lowByte;       // 12-bit raw angle
  float currentAngle = (currentRawAngle / 4095.0) * 360.0; // Convert raw angle to degrees

  unsigned long currentMicros = micros();

  // Initialize time and angle
  if (previousMicros == 0)
  {
    previousMicros = currentMicros;
    previousAngle = currentAngle;
    return 0.0;
  }

  float dt = (currentMicros - previousMicros) / 1000000.0;
  if (dt == 0)
    return 0.0;

  float deltaAngle = currentAngle - previousAngle;

  // Adjust for wrapping
  if (deltaAngle > 180)
    deltaAngle -= 360;
  else if (deltaAngle < -180)
    deltaAngle += 360;

  // Calculate angular velocity in degrees per second
  float angularVelocity = deltaAngle / dt;

  // Convert angular velocity from degrees per second to RPM
  float rpm = angularVelocity / 360.0 * 60.0;

  // Update previous values
  previousAngle = currentAngle;
  previousMicros = currentMicros;

  return rpm;
}

int setFTH(TwoWire &wire, byte value)
{
  if (value > 0x07)
  {
    return COM_FAIL; // Invalid value for FTH (must be 0-7)
  }

  // Read current CONF register value
  wire.beginTransmission(AS5600_ADDR);
  wire.write(CONF_ADDR);
  wire.endTransmission(false);
  wire.requestFrom(AS5600_ADDR, 2);
  if (wire.available() != 2)
  {
    return COM_FAIL;
  }

  byte msb = wire.read(); // Read high byte of CONF
  byte lsb = wire.read(); // Read low byte of CONF

  // Clear current FTH bits and set new value
  lsb = (lsb & ~FTH_MASK) | (value << 2);

  // Write new CONF register value
  wire.beginTransmission(AS5600_ADDR);
  wire.write(CONF_ADDR);
  wire.write(msb); // Write high byte of CONF
  wire.write(lsb); // Write modified low byte of CONF
  if (wire.endTransmission() != 0)
  {
    return COM_FAIL;
  }

  return COM_SUCCESS;
}