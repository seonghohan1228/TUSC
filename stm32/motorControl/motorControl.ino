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

// LED pins
const int LEDPin0 = PB6;
const int LEDPin1 = PA4;

// Pulsewidth; units are in ms
const int MIN_PULSEWIDTH = 1000;
const int IDLE_PULSEWIDTH = 1500;
const int MAX_PULSEWIDTH = 2000;

// Maxmum velocity in rpm
const int MAX_VELOCITY = 5000;

// remote Input range
const int INPUT_RANGE = 100;

float targetSpeed1 = 0;
float targetSpeed2 = 0;
float currentSpeed1 = 0;
float currentSpeed2 = 0;

float stop_threshold = 30;

uint8_t status = 1; // 0: OFF, 1: ON
uint8_t pid = 1; // 0: DISABLED, 1: ENABLED

// ESC instances
Servo ESC_L;
Servo ESC_R;

// I2C instances
TwoWire sen1(PB7, PA15);
TwoWire sen2(PB5, PA8);

// PID instances
PID pid1(KP1, KI1, KD1, MIN_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);
PID pid2(KP2, KI2, KD2, MIN_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);


void slipDetection()
{
  if (pid == 1 && status == 1)
  {
    digitalWrite(LEDPin0, HIGH);
    digitalWrite(LEDPin1, HIGH);
  }
  else
  {
    digitalWrite(LEDPin0, LOW);
    digitalWrite(LEDPin1, LOW);
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

  // PWM pins for ESCs
  ESC_L.attach(PA0, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(PA1, MIN_PULSEWIDTH, MAX_PULSEWIDTH);

  pinMode(LEDPin0, OUTPUT);
  pinMode(LEDPin1, OUTPUT);

  // Set speed to 0 (Idle)
  ESC_L.writeMicroseconds(IDLE_PULSEWIDTH);
  ESC_R.writeMicroseconds(IDLE_PULSEWIDTH);
}

void loop()
{
  static byte buffer[PACKET_SIZE];
  static int buffer_index = 0;

  static float pwmValue1 = IDLE_PULSEWIDTH;
  static float pwmValue2 = IDLE_PULSEWIDTH;

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
          status = packet[1]; // 0: OFF, 1: ON
          pid = packet[2]; // 0: DISABLED, 1: ENABLED

          int16_t speed_L = (packet[3] << 8) | packet[4]; // Higher byte | Lower byte
          int16_t speed_R = (packet[5] << 8) | packet[6];

          // Convert input speed (-100 ~ 100) to rpm (-5000 ~ 5000)
          speed_L = map(speed_L, -INPUT_RANGE, INPUT_RANGE, -MAX_VELOCITY, MAX_VELOCITY);
          speed_R = map(speed_R, -INPUT_RANGE, INPUT_RANGE, -MAX_VELOCITY, MAX_VELOCITY);

          currentSpeed1 = pid1.readRPM(sen1);
          currentSpeed2 = pid2.readRPM(sen2);
          
          pid1.goalVelocity(speed_L);
          pid2.goalVelocity(speed_R);

          pwmValue1 = pid1.computePulseWidth(currentSpeed1);
          pwmValue2 = pid2.computePulseWidth(currentSpeed2);

          delay(2); // Due to encoder sampling rate

          if (pid == 0)
          {
            pwmValue1 = map(speed_L, -MAX_VELOCITY, MAX_VELOCITY, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
            pwmValue2 = map(speed_R, -MAX_VELOCITY, MAX_VELOCITY, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
          }

          if (status == 0) // Turn off
          {
            pwmValue1 = IDLE_PULSEWIDTH;
            pwmValue2 = IDLE_PULSEWIDTH;
            ESC_L.writeMicroseconds(pwmValue1);
            ESC_R.writeMicroseconds(pwmValue2);
            slipDetection();
            while(1) {}
          }

          ESC_L.writeMicroseconds(pwmValue1);
          ESC_R.writeMicroseconds(pwmValue2);
          
          Serial.print(status);
          Serial.print(" ");
          Serial.print(pid);
          Serial.print(" ");
          Serial.print(speed_L);
          Serial.print(" ");
          Serial.print(pid1.get_Velocity());
          Serial.print(" ");
          Serial.print(pwmValue1);
          Serial.print("\t");
          Serial.print(speed_R);
          Serial.print(" ");
          Serial.print(pid2.get_Velocity());
          Serial.print(" ");
          Serial.println(pwmValue2);

          slipDetection();
        }

        buffer_index = 0;
      }
    }
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
