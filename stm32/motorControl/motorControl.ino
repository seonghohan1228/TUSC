#include <Wire.h>
#include <Servo.h>
#include <queue>

#include "packet.h"

// Modes
const int STEER = 0;
const int TANK = 1;

// AS5600 I2C address
#define AS5600_ADDR 0x36
#define ANGLE_ADDR 0x0E
#define ZPOSH_ADDR 0x01
#define ZPOSL_ADDR 0x02
#define STATUS_ADDR 0x0B

#define COM_FAIL -1
#define COM_SUCCESS 1

// Pin numbers
const int ModeLEDPin0 = PA9;
const int ModeLEDPin1 = PA10;
const int GearLEDPin0 = PA4;
const int GearLEDPin1 = PA5;
const int GearLEDPin2 = PA6;
const int GearLEDPin3 = PA7;

// Pulsewidth; units are in ms
const int MIN_PULSEWIDTH = 1000;
const int IDLE_PULSEWIDTH = 1500;
const int MAX_PULSEWIDTH = 2000;

// ESC instances
Servo ESC_L;    
Servo ESC_R;

// I2C instances
TwoWire sen1(PB7, PA15);
TwoWire sen2(PB5, PA8);

/* ENCODER ********************************************************************/
float readAngle(TwoWire &wire)
{
  /*
  description : read angle in degree
  @param &wire  (TwoWire) sensor
  @return : (float) degrees
  */
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


bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition)
{
  /*
  description : set zero position
  @param &wire  (TwoWire) sensor
  @param zeroPosition  (unsigned int)  the zero position value (12 bits)
  @return (bool) success
  */
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


byte readStatus(TwoWire &wire)
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
/******************************************************************************/
/* PACKET *********************************************************************/
uint8_t calculate_checksum(uint8_t data[], int length)
{
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++)
  {
    checksum += data[i];
  }
  return checksum % 256;
}


void LED_control(int mode, int gear)
{
  switch (mode)
  {
    case STEER:
      digitalWrite(ModeLEDPin0, HIGH);
      digitalWrite(ModeLEDPin1, LOW);
      break;
    case TANK:
      digitalWrite(ModeLEDPin0, LOW);
      digitalWrite(ModeLEDPin1, HIGH);
      break;
    default:
      // Mode error
      break;
  }

  switch (gear)
  {
    case 1:
      digitalWrite(GearLEDPin0, HIGH);
      digitalWrite(GearLEDPin1, LOW);
      digitalWrite(GearLEDPin2, LOW);
      digitalWrite(GearLEDPin3, LOW);
      break;
    case 2:
      digitalWrite(GearLEDPin0, HIGH);
      digitalWrite(GearLEDPin1, HIGH);
      digitalWrite(GearLEDPin2, LOW);
      digitalWrite(GearLEDPin3, LOW);
      break;
    case 3:
      digitalWrite(GearLEDPin0, HIGH);
      digitalWrite(GearLEDPin1, HIGH);
      digitalWrite(GearLEDPin2, HIGH);
      digitalWrite(GearLEDPin3, LOW);
      break;
    case 4:
      digitalWrite(GearLEDPin0, HIGH);
      digitalWrite(GearLEDPin1, HIGH);
      digitalWrite(GearLEDPin2, HIGH);
      digitalWrite(GearLEDPin3, HIGH);
      break;
    default:
      // Speed level error
      break;
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup()
{
  // Serial connection
  Serial.begin(115200);

  // Encoders
  sen1.begin();
  sen2.begin();

  // LED pins
  pinMode(ModeLEDPin0, OUTPUT);
  pinMode(ModeLEDPin1, OUTPUT);
  pinMode(GearLEDPin0, OUTPUT);
  pinMode(GearLEDPin1, OUTPUT);
  pinMode(GearLEDPin2, OUTPUT);
  pinMode(GearLEDPin3, OUTPUT);
  
  // PWM pins for ESCs
  ESC_L.attach(PA0, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(PA1, MIN_PULSEWIDTH, MAX_PULSEWIDTH);

  // Set speed to 0 (Idle)
  ESC_L.writeMicroseconds(IDLE_PULSEWIDTH);
  ESC_R.writeMicroseconds(IDLE_PULSEWIDTH);
}

void loop()
{
  static byte buffer[PACKET_SIZE];
  static int buffer_index = 0;

  // Read angle from sensors
  float angle1 = readAngle(sen1);
  float angle2 = readAngle(sen2);
  
  while (Serial.available())
  {
    byte incoming_byte = Serial.read();

    if (buffer_index == 0)
      if (incoming_byte == START_BYTE)
        buffer[buffer_index++] = incoming_byte;
    
    else
    {
      buffer[buffer_index++] = incoming_byte;
      if (buffer_index == PACKET_SIZE)
      {
        // Read incoming bytes
        uint8_t received[PACKET_SIZE];
        Serial.readBytes(received, PACKET_SIZE);
        
        // Check validity of packet
        struct Packet *packet;
        arrayToPacket(received, packet);
        if (packetIsValid(packet));
        {
          uint8_t mode = packet->payload->mode;
          uint8_t gear = packet->payload->gear;
          uint16_t speed_L = packet->payload->speed_L;
          uint16_t speed_R = packet->payload->speed_R;
          
          // Control
          LED_control(mode, gear);

          ESC_L.write(map(speed_L, -100, 100, 0, 180));
          ESC_R.write(map(speed_R, -100, 100, 0, 180));
        }

        buffer_index = 0;
      }
    }
  
    /*
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
    }*/
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

