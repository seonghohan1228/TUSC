#include <Servo.h>

int ESCPin_L = 9;
int ESCPin_R = 10;

// Pulsewidth units are in ms
int MIN_PULSEWIDTH = 1000;
int IDLE_PULSEWIDTH = 1500;
int MAX_PULSEWIDTH = 2000;
int DEADBAND = 5;

int inputSpeed[2];

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
  if (Serial.available() >= 2) {
    for (int i = 0; i < 2; i++){
      inputSpeed[i] = Serial.read();
      inputSpeed[i] -= 100;  // To fit range of 0~255
    }
  }
  int brightness = map(inputSpeed[0], -100, 100, 0, 255);
  analogWrite(5, brightness);
  
  ESC_L.write(map(inputSpeed[0], -100, 100, 0, 180));
  ESC_R.write(map(inputSpeed[1], -100, 100, 0, 180));
}


void printValues(int value_L, int value_R)
{
  Serial.print("L: ");
  Serial.print(value_L);
  Serial.print("\t");
  Serial.print("R: ");
  Serial.println(value_R);
}
