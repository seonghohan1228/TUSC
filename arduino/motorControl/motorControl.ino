#include <Servo.h>

int ESCPin_L = 9;
int ESCPin_R = 10;

// Pulsewidth units are in ms
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
  pinMode(5, OUTPUT);

  ESC_L.attach(ESCPin_L, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(ESCPin_R, MIN_PULSEWIDTH, MAX_PULSEWIDTH);

}


void loop()
{
  if (Serial.available() >= 3) {
    for (int i = 0; i < 3; i++){
      incoming[i] = Serial.read();
    }
  }
  int gear = incoming[0];
  int speed_L = incoming[1] - 100;  // Speed was shifted +100 when sending from Pi (0~200)
  int speed_R = incoming[2] - 100;
  
  ESC_L.write(map(speed_L, -100, 100, 0, 180));
  ESC_R.write(map(speed_R, -100, 100, 0, 180));
}

