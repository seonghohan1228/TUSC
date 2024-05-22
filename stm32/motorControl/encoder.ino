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
  wire.write(ZPOSH_ADDR);          // AS5600 register 0x01 for high byte of ZPOS
  wire.write(highByte);            // Write high byte of zero position
  wire.write(ZPOSL_ADDR);          // AS5600 register 0x02 for low byte of ZPOS
  wire.write(lowByte);             // Write low byte of zero position

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
