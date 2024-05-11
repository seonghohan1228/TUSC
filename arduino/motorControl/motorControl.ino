#include <Servo.h>

int ESCPin_L = 5;
int ESCPin_R = 6;  /* TODO: Find different PWM out pin */
int inputPin_L = 9;
int inputPin_R = 10;  /* TODO: Find different PWM in pin */

int inputValue_L = 0;
int inputValue_R = 0;

// Pulsewidth units are in ms
int MIN_PULSEWIDTH = 1000;
int IDLE_PULSEWIDTH = 1500;
int MAX_PULSEWIDTH = 2000;

int pulsewidth_L;
int pulsewidth_R;

Servo ESC_L;    // Create servo object to control the ESC
Servo ESC_R;

void setup()
{
  Serial.begin(115200);
  Serial.println("\nSerial connection started.\n");
  delay(500);

  ESC_L.attach(ESCPin_L, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  ESC_R.attach(ESCPin_R, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
  pinMode(inputPin_L, INPUT);
  pinMode(inputPin_R, INPUT);
}

void loop()
{
  /*
  static int currentTime = 0;
  Serial.print("Time interval: ");
  Serial.print(millis() - currentTime);
  Serial.println(" ms");
  currentTime = millis();*/

  // Read PWM signals sent from Pi
  int pulseDuration_L = pulseIn(inputPin_L, HIGH);  // Measures duration of HIGH signal
  int pulseDuration_R = pulseIn(inputPin_R, HIGH);

  // Set PWM output for ESCs
  int servoValue_L = map(pulseDuration_L, MIN_PULSEWIDTH, MAX_PULSEWIDTH, 0, 180);
  int servoValue_R = map(pulseDuration_R, MIN_PULSEWIDTH, MAX_PULSEWIDTH, 0, 180);

  ESC_L.write(servoValue_L);
  ESC_R.write(servoValue_R);
  
  printValues(pulseDuration_L, pulseDuration_R);
}


void printValues(int value_L, int value_R)
{
  Serial.print("L: ");
  Serial.print(value_L);
  Serial.print("\t");
  Serial.print("R: ");
  Serial.println(value_R);
}
