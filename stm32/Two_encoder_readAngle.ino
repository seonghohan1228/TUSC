#include <Wire.h>
#include <Servo.h>

TwoWire sen1(PB7, PB6); // I2C instance
TwoWire sen2(PB11, PB10);
// HardwareSerial UART2(PA10, PA9); // (RX, TX)

// AS5600 I2C address
#define AS5600_ADDR 0x36
#define ANGLE_ADDR 0x0E
#define ZPOSH_ADDR 0x01
#define ZPOSL_ADDR 0x02
#define STATUS_ADDR 0x0B

#define COM_FAIL -1
#define COM_SUCCESS 1

/*
description : read angle in degree
@param &wire  (TwoWire) sensor
@return : (float) degrees
*/
float readAngle(TwoWire &wire)
{
  wire.beginTransmission(AS5600_ADDR);
  wire.write(ANGLE_ADDR);      // AS5600 register 0x0E for angle
  wire.endTransmission(false); // Don't end transmission for continuous sending

  wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes for angle data
  if (wire.available() >= 2)
  {
    byte highByte = wire.read(); // Read high byte
    byte lowByte = wire.read();  // Read low byte

    return ((highByte << 8) | lowByte) * 0.0879; // Convert to angle in degrees
  }
  return COM_FAIL; // Return FAIL if reading fails
}

/*
description : set zero position
@param &wire  (TwoWire) sensor
@param zeroPosition  (unsigned int)  the zero position value (12 bits)
@return (bool) success
*/
bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition)
{
  if (zeroPosition > 4095) // Check if the zeroPosition is within 12-bit range
    return COM_FAIL;       // Return false if zeroPosition is out of range

  byte highByte = zeroPosition >> 8;  // Extract high byte (higher 4 bits)
  byte lowByte = zeroPosition & 0xFF; // Extract low byte (lower 8 bits)

  wire.beginTransmission(AS5600_ADDR);
  wire.write(ZPOSH_ADDR);          // AS5600 register 0x01 for high byte of ZPOS
  wire.write(highByte);            // Write high byte of zero position
  wire.write(ZPOSL_ADDR);          // AS5600 register 0x02 for low byte of ZPOS
  wire.write(lowByte);             // Write low byte of zero position
  if (wire.endTransmission() == 0) // Check if transmission was successful
    return COM_SUCCESS;            // Return true if transmission succeeds
  else
    return COM_FAIL; // Return false if transmission fails
}

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
byte readStatus(TwoWire &wire)
{
  byte status;
  byte result = 0; // Initialize result

  wire.beginTransmission(AS5600_ADDR);
  wire.write(0x0B); // STATUS register address
  wire.endTransmission();
  wire.requestFrom(AS5600_ADDR, 1); // Request 1 byte

  if (wire.available())
    status = wire.read(); // Read status register
  else
    return result; // If no data is available, return 0

  // Extract bits from the status byte
  bool md = status & 0x01; // Magnet detected
  bool ml = status & 0x02; // Magnet too low
  bool mh = status & 0x04; // Magnet too high
  bool co = status & 0x08; // Cordic overflow

  // Assign bits to the result
  result |= (md << 0); // Set bit 0 for MD
  result |= (ml << 1); // Set bit 1 for ML
  result |= (mh << 2); // Set bit 2 for MH
  result |= (co << 3); // Set bit 3 for CO

  // Set the 5th bit if no special error (ML, MH, CO) is detected
  if (!ml && !mh && !co)
    result |= (1 << 4); // Set bit 4 if no errors

  return result; // Return the 5-bit status representation
}

// Pulsewidth units are in ms
int MIN_PULSEWIDTH = 1000;
int IDLE_PULSEWIDTH = 1500;
int MAX_PULSEWIDTH = 2000;
int DEADBAND = 5;

int inputSpeed[2];

int servoValue_L;
int servoValue_R;

void setup()
{
  // Serial.begin(115200)5;
  Serial.begin(115200, SERIAL_8E1);
  sen1.begin(); // Start software I2C
  sen2.begin();
  // ESC_L.attach(PA8, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  // ESC_R.attach(PA9, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
}

void loop()
{
  float angle1 = readAngle(sen1); // 첫 번째 센서에서 각도 읽기
  float angle2 = readAngle(sen2); // 두 번째 센서에서 각도 읽기
  // String data = Serial.readStringUntil('\n');
  // Serial.println("Receved STM32 = " + data);
  // if (UART2.available() > 0) {
  if (angle1 != COM_FAIL)
  {
    Serial.print(angle1);
    Serial.print(" ");
  }
  else
  {
    Serial.print("Failed to read angle from sensor 1; ");
  }

  if (angle2 != COM_FAIL)
  {
    Serial.println(angle2);
  }
  else
  {
    Serial.println("Failed to read angle from sensor 2");
  }
}