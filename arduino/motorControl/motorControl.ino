#include <Servo.h>

// Modes
#define STEER 0
#define TANK 1

// Pin numbers
int ModeLEDPin0 = 2;
int ModeLEDPin1 = 3;
int GearLEDPin0 = 4;
int GearLEDPin1 = 5;
int GearLEDPin2 = 6;
int GearLEDPin3 = 7;

int ESCPin_L = 9;
int ESCPin_R = 10;

// Pulsewidths (unit: us)
int MIN_PULSEWIDTH = 1000;
int IDLE_PULSEWIDTH = 1500;
int MAX_PULSEWIDTH = 2000;

// Packet info
const byte START_BYTE = 0x02;
const byte END_BYTE = 0x03;
const int PACKET_SIZE = 9;

int servoValue_L;
int servoValue_R;

Servo ESC_L;    // Create servo object to control the ESC
Servo ESC_R;


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
  Serial.begin(9600);
  // LED pins
  pinMode(ModeLEDPin0, OUTPUT);
  pinMode(ModeLEDPin1, OUTPUT);
  pinMode(GearLEDPin0, OUTPUT);
  pinMode(GearLEDPin1, OUTPUT);
  pinMode(GearLEDPin2, OUTPUT);
  pinMode(GearLEDPin3, OUTPUT);

  // PWM pins for ESCs
  ESC_L.attach(ESCPin_L, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(ESCPin_R, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
}


void loop()
{
  static byte buffer[PACKET_SIZE];
  static int buffer_index = 0;

  while (Serial.available())
  {
    byte incoming_byte = Serial.read();

    if (buffer_index == 0)
    {
      if (incoming_byte == START_BYTE)
      {
        buffer[buffer_index++] = incoming_byte;
      }
    }
    else
    {
      buffer[buffer_index++] = incoming_byte;
      if (buffer_index == PACKET_SIZE)
      {
        // Read incoming bytes
        byte received[PACKET_SIZE];
        Serial.readBytes(received, PACKET_SIZE);
        // Check validity of packet
        if (received[0] == START_BYTE && received[PACKET_SIZE - 1] == END_BYTE)
        {
          byte mode = received[1];
          byte gear = received[2];
          int16_t speed_L = (received[3] << 8) | received[4];  // Higher byte | Lower byte
          int16_t speed_R = (received[5] << 8) | received[6];

          byte payload[] = { mode, gear, received[3], received[4], received[5], received[6] };

          // If checksum is correct, act
          byte checksum = calculate_checksum(payload, 6);
          if (checksum == received[7])
          {
            LED_control(mode, gear);

            ESC_L.write(map(speed_L, -100, 100, 0, 180));
            ESC_R.write(map(speed_R, -100, 100, 0, 180));
          }
        }
        buffer_index = 0;
      }
    }
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

