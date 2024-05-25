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

float stop_threshold = 30;

uint8_t tuscOn = ON;
uint8_t pidEnable = ENABLED;

float leftPWM = IDLE_PULSEWIDTH;
float rightPWM = IDLE_PULSEWIDTH;

int16_t inputLeftSpeed = 0;
int16_t inputRightSpeed = 0;
int16_t targetLeftSpeed = 0;
int16_t targetRightSpeed = 0;
int16_t currentLeftSpeed = 0;
int16_t currentRightSpeed = 0;

// ESC instances
Servo ESC_L;
Servo ESC_R;

// I2C instances
TwoWire sen1(PB7, PA15);
TwoWire sen2(PB5, PA8);

// PID instances
PID pid1(KP1, KI1, KD1, IDLE_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);
PID pid2(KP2, KI2, KD2, IDLE_PULSEWIDTH, MAX_PULSEWIDTH, IDLE_PULSEWIDTH);

void ledControl()
{
  if (!pidEnable)
  {
    digitalWrite(LEDPin0, LOW);
    digitalWrite(LEDPin1, LOW);
  }
  else
  {
    if (leftPWM == IDLE_PULSEWIDTH || leftPWM == MAX_PULSEWIDTH)
      digitalWrite(LEDPin0, HIGH);
    else
      digitalWrite(LEDPin0, LOW);

    if (rightPWM == IDLE_PULSEWIDTH || rightPWM == MAX_PULSEWIDTH)
      digitalWrite(LEDPin1, HIGH);
    else
      digitalWrite(LEDPin1, LOW);
  }
  if (!tuscOn)
  {
    digitalWrite(LEDPin0, LOW);
    digitalWrite(LEDPin1, LOW);
  }
}

void sendStatus()
{
  // Format the data as a compact string
  // Example format: "1 1 150 145 150 140\n"
  Serial.print(tuscOn);
  Serial.print(" ");
  Serial.print(pidEnable);
  Serial.print(" ");
  Serial.print(targetLeftSpeed);
  Serial.print(" ");
  Serial.print(currentLeftSpeed);
  Serial.print(" ");
  Serial.print(targetRightSpeed);
  Serial.print(" ");
  Serial.print(currentRightSpeed);
  Serial.println();
}

// timer interrupt handler
void TIM_IT_Handler()
{
  if (pid1.readRPM(sen1) == COM_FAIL)
    NVIC_SystemReset();
  if (pid2.readRPM(sen2) == COM_FAIL)
    NVIC_SystemReset();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup()
{
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait for the serial port to be ready
  while (!Serial)
    ;

  // Encoders
  sen1.setClock(400000);
  sen2.setClock(400000);
  sen1.begin();
  sen2.begin();
  if (setFTH(sen1, 0x07) == COM_SUCCESS) // Encoder filter setting
    Serial.println("FTH set successfully");
  else
    Serial.println("Failed to set FTH");
  if (setFTH(sen2, 0x07) == COM_SUCCESS)
    Serial.println("FTH set successfully");
  else
    Serial.println("Failed to set FTH");

  // PWM pins for ESCs
  ESC_L.attach(PA0, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(PA1, MIN_PULSEWIDTH, MAX_PULSEWIDTH);

  pinMode(LEDPin0, OUTPUT);
  pinMode(LEDPin1, OUTPUT);

  // Set speed to 0 (Idle)
  ESC_L.writeMicroseconds(IDLE_PULSEWIDTH);
  ESC_R.writeMicroseconds(IDLE_PULSEWIDTH);

  // Set timer interrupt
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1000, HERTZ_FORMAT); // set to 1kHz sampling
  MyTim->attachInterrupt(TIM_IT_Handler);
  MyTim->resume();
}

void loop()
{
  static byte buffer[PACKET_SIZE];
  static int bufferIndex = 0;

  while (Serial.available())
  {
    byte incomingByte = Serial.read();

    if (bufferIndex == 0)
    {
      if (incomingByte == START_BYTE)
      {
        buffer[bufferIndex++] = incomingByte;
      }
    }

    else
    {
      buffer[bufferIndex++] = incomingByte;

      if (bufferIndex == PACKET_SIZE)
      {
        // Read incoming bytes
        uint8_t packet[PACKET_SIZE];
        Serial.readBytes(packet, PACKET_SIZE);

        // Check validity of packet
        if (packetIsValid(packet))
        {
          tuscOn = packet[1];
          pidEnable = packet[2];

          inputLeftSpeed = (packet[3] << 8) | packet[4];
          inputRightSpeed = (packet[5] << 8) | packet[6];

          if (!tuscOn) // TUSC is turned off
          {
            ESC_L.writeMicroseconds(IDLE_PULSEWIDTH);
            ESC_R.writeMicroseconds(IDLE_PULSEWIDTH);
            sendStatus();
            ledControl();

            // Reset STM32
            NVIC_SystemReset();
          }

          targetLeftSpeed = map(inputLeftSpeed, -INPUT_RANGE, INPUT_RANGE, -MAX_VELOCITY, MAX_VELOCITY);
          targetRightSpeed = map(inputRightSpeed, -INPUT_RANGE, INPUT_RANGE, -MAX_VELOCITY, MAX_VELOCITY);

          currentLeftSpeed = pid1.get_Velocity();
          currentRightSpeed = pid2.get_Velocity();

          pid1.goalVelocity(targetLeftSpeed);
          pid2.goalVelocity(targetRightSpeed);

          if (pidEnable)
          {
            leftPWM = pid1.computePulseWidth();
            rightPWM = pid2.computePulseWidth();
          }
          else
          {
            leftPWM = map(currentLeftSpeed, -MAX_VELOCITY, MAX_VELOCITY, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
            rightPWM = map(currentLeftSpeed, -MAX_VELOCITY, MAX_VELOCITY, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
          }

          ESC_L.writeMicroseconds(leftPWM);
          ESC_R.writeMicroseconds(rightPWM);

          sendStatus();
          ledControl();
        }

        bufferIndex = 0;
      }
    }
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
