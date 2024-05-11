#include <Servo.h>

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;

volatile unsigned long pwm_value[2] = {0, 0};
volatile unsigned long timer[2] = {0, 0};

int ESCPin_L = 5;
int ESCPin_R = 6;

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

  // When external interrupt pin 2, 3 go high, call the rising function
  attachInterrupt(digitalPinToInterrupt(interruptPin1), calcPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), calcPWM2, CHANGE);
}

void loop()
{
  /*
  static int currentTime = 0;
  Serial.print("Time interval: ");
  Serial.print(millis() - currentTime);
  Serial.println(" ms");
  currentTime = millis();*/

  // Set PWM output for ESCs
  int servoValue_L = map(pwm_value[0], MIN_PULSEWIDTH, MAX_PULSEWIDTH, 0, 180);
  int servoValue_R = map(pwm_value[1], MIN_PULSEWIDTH, MAX_PULSEWIDTH, 0, 180);

  ESC_L.write(servoValue_L);
  ESC_R.write(servoValue_R);
  
  printValues(pwm_value[0], pwm_value[1]);
}


void calcPWM1() 
{
  if(digitalRead(interruptPin1) == HIGH) { 
    timer[0] = micros();
    } 
  else {
    if(timer[0] != 0) {
      pwm_value[0] = micros() - timer[0];
      }
    } 
} 

void calcPWM2() 
{
  if(digitalRead(interruptPin2) == HIGH) { 
    timer[1] = micros();
    } 
  else {
    if(timer[1] != 0) {
      pwm_value[1] = micros() - timer[1];
      }
    } 
} 


void printValues(int value_L, int value_R)
{
  Serial.print("L: ");
  Serial.print(value_L);
  Serial.print("\t");
  Serial.print("R: ");
  Serial.println(value_R);
}
