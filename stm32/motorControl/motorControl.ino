// motorControl.ino

#include <Wire.h>
#include <Servo.h>
#include <queue>

#include "encoder.h"
#include "packet.h"
#include "pid.hpp"

// Modes
const int STEER = 0;
const int TANK = 1;

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

float targetSpeed1 = 3000.0;
float targetSpeed2 = 3000.0;
float currentSpeed1 = 0;
float currentSpeed2 = 0;

float stop_threshold = 30;

// ESC instances
Servo ESC_L;    
Servo ESC_R;

// I2C instances
TwoWire sen1(PB7, PA15);
TwoWire sen2(PB5, PA8);

// PID instances
MovingAverageFilter rpmFilter(30);
PID pid1(KP1, KI1, KD1, MIN_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);
PID pid2(KP2, KI2, KD2, MIN_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);

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

  static float pwmValue1;
  static float pwmValue2;
  
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
        uint8_t packet[PACKET_SIZE];
        Serial.readBytes(packet, PACKET_SIZE);
        
        // Check validity of packet
        if (packetIsValid(packet))
        {
          uint8_t mode = packet[1];
          uint8_t gear = packet[2];
          int16_t speed_L = (packet[3] << 8) | packet[4];  // Higher byte | Lower byte
          int16_t speed_R = (packet[5] << 8) | packet[6];
          
          pid1.goalVelocity(speed_L);
          pid2.goalVelocity(speed_R);
          
          currentSpeed1 = readRPM(sen1);
          currentSpeed2 = readRPM(sen2);
          pwmValue1 = pid1.computePulseWidth(currentSpeed1);
          pwmValue2 = pid2.computePulseWidth(currentSpeed2);
          
          // Stabilizing stop
          if (abs(targetSpeed1) < stop_threshold)
            ESC_L.writeMicroseconds(IDLE_PULSEWIDTH);
          if (abs(targetSpeed2) < stop_threshold)
            ESC_R.writeMicroseconds(IDLE_PULSEWIDTH);
          
          
          // Control
          LED_control(mode, gear);

          ESC_L.writeMicroseconds(pwmValue1);
          ESC_R.writeMicroseconds(pwmValue2);
        }

        buffer_index = 0;
      }
    }
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

