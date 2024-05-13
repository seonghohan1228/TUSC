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

int incoming[3];

int servoValue_L;
int servoValue_R;

Servo ESC_L;    // Create servo object to control the ESC
Servo ESC_R;


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
  if (Serial.available() >= 4) {
    for (int i = 0; i < 4; i++){
      incoming[i] = Serial.read();
    }

    int mode = incoming[0];
    int gear = incoming[1];
    int speed_L = incoming[2] - 100;  // Speed was shifted +100 when sending from Pi (0~200)
    int speed_R = incoming[3] - 100;
    
    LED_control(mode, gear);

    ESC_L.write(map(speed_L, -100, 100, 0, 180));
    ESC_R.write(map(speed_R, -100, 100, 0, 180));
  }
  
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

